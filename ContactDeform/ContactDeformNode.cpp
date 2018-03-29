//
// Copyright (C) 2018 Michael Abrahams
// 
// File: ContactDeformNode.cpp
//
// Implements a simple Maya deformer node deforming two meshes based on contact proximity.
//
// Based on jlCollisionDeformer.py by Jan Lachauer.
//


#include "ContactDeformNode.h"

#include <vector>
#include <string.h>
#include <math.h>

#include <maya/MGlobal.h>
#include <maya/MIOStream.h>
#include <maya/MPxGeometryFilter.h>
#include <maya/MItGeometry.h>
#include <maya/MFloatPointArray.h>
#include <maya/MFloatArray.h>
#include <maya/MMeshIntersector.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MFloatMatrix.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>
#include <maya/MItMeshVertex.h>  
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnGenericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MRampAttribute.h>


#define MCheckErr(stat,msg)		\
if ( MS::kSuccess != stat ) {	\
	cerr << msg;				\
	return MS::kFailure;		\
}

#define PrintTriple(v) (MString("(") + v[0] + ", " + v[1] + ", " + v[2] + ")")

struct ContactDeformPrivate {
	MMeshIsectAccelParams mmAccelParams;
	MMeshIntersector intersector;
	MFloatPointArray newpoints;
	MFloatPointArray baseColliderPoints;
};


// You MUST change this to a unique value!!!  The id is a 32bit value used
// to identify this type of node in the binary file format.  
//
MTypeId     ContactDeform::id( 0x0010A52C );

// Declare static attributes here.
// If you get linker errors, this is the reason!
// 
MObject     ContactDeform::collider;
MObject     ContactDeform::bulgeextend;
MObject     ContactDeform::bulge;
MObject     ContactDeform::offset;
MObject     ContactDeform::colliderBBoxX;
MObject     ContactDeform::colliderBBoxY;
MObject     ContactDeform::colliderBBoxZ;
MObject     ContactDeform::colliderBBoxSize;
MObject     ContactDeform::colliderMatrix;
MObject     ContactDeform::bulgeshape;
MObject     ContactDeform::backface;
MObject     ContactDeform::sculptmode;
    



ContactDeform::ContactDeform() {
	d = new ContactDeformPrivate();
}

ContactDeform::~ContactDeform() {
	delete d;
}


// TODO: See if this can be implemented as deform() instead
MStatus ContactDeform::compute( const MPlug& plug, MDataBlock& dataBlock )
//
//	Description:
//		This method computes the value of the given output plug based
//		on the values of the input attributes.
//
//	Arguments:
//		plug - the plug to compute
//		data - object that provides access to the attributes for this node
//
{
	MStatus returnStatus;
 

	// Check which output attribute we have been asked to compute.  If this 
	// node doesn't know how to compute it, we must return 
	// MS::kUnknownParameter.
	if (plug.attribute() != outputGeom) {
		MGlobal::displayError("Unknown output parameter requested!\n");
		return MS::kUnknownParameter;
	}

	// Get a handle to the input attribute that we will need for the
	// computation.  If the value is being supplied via a connection 
	// in the dependency graph, then this call will cause all upstream  
	// connections to be evaluated so that the correct value is supplied.
	// 
	MDataHandle inputData = dataBlock.inputValue( input, &returnStatus );

	MCheckErr(returnStatus, "Node ContactDeform cannot get value\n");


	// TODO: See if we can add this somewhere safely. Good for DG health.
	// dataBlock.setClean(plug);


	MVector				vector;
	MVector				normvector;
	MPoint				point;
	MVector				pointNormal;
	MVector				colliderPointNormal;
	MVector				distanceVector;
	MFloatPointArray	deformedPoints;
	std::vector<int>	deformedPointsIndices;
	MPointOnMesh		pointInfo;
	MFloatArray			emptyFloatArray;
	MVector				inMeshNormal;
	std::vector<int>	connectedPnts;
	int					maxDistance = 0;
	MFloatPoint			worldPoint;
	MPointOnMesh		indirPointInfo;

	// Handles
	MDataHandle envelopeHandle = dataBlock.inputValue(MPxGeometryFilter::envelope);
	float envelopeValue = envelopeHandle.asFloat();

	MDataHandle offsetHandle = dataBlock.inputValue(offset);
	double offsetValue = offsetHandle.asDouble();

	MDataHandle bulgeExtendHandle = dataBlock.inputValue(bulgeextend);
	double bulgeExtendValue = bulgeExtendHandle.asDouble();

	MDataHandle bulgeHandle = dataBlock.inputValue(bulge);
	double bulgeValue = bulgeHandle.asDouble();

	// Hmm, says this stuff is for Python only?
	// bulgeshapeValueUtil = MScriptUtil();
	// bulgeshapeValue = bulgeshapeValueUtil.asFloatPtr()

	// A bit sketchy but seems OK for now
	MObject thisNode = this->thisMObject();
	MRampAttribute bulgeshapeHandle(thisNode, bulgeshape);

	MDataHandle sculptHandle = dataBlock.inputValue(sculptmode);
	short sculptValue = sculptHandle.asShort();

	MDataHandle backfaceHandle = dataBlock.inputValue(backface);
	short backfaceValue = backfaceHandle.asShort();

	// MGlobal::displayInfo(MString("Envelope: ") + envelopeValue + "\nOffset: " + offsetValue + "\nBulge Extend: " + bulgeExtendValue \
	//					 + "\nBulge: " + bulgeValue + "\nSculpt: " + sculptValue + "\nBackface: " + backfaceValue);


	// Input object data
	MArrayDataHandle hInput = dataBlock.inputArrayValue(MPxGeometryFilter::input);
	unsigned int multiIndex = plug.logicalIndex(); // deformer handle
	hInput.jumpToArrayElement(multiIndex);
	MDataHandle hInputElement = hInput.inputValue(&returnStatus);
	MCheckErr(returnStatus, "Could not find input element.\n");

	// Input geometry data
	MDataHandle hInputGeom = hInputElement.child(MPxGeometryFilter::inputGeom);
	MObject inMesh = hInputGeom.asMesh();
	MFnMesh inMeshFn(inMesh);

	MFloatPointArray inPoints, colliderPoints;
	inMeshFn.getPoints(inPoints, MSpace::kWorld);


	// copy world space points into array to set them later. 
	// The Python version used copy() which I think is a shallow copy?
	if ((d->newpoints.length() == 0) || (sculptValue == 0))
		d->newpoints.copy(inPoints);

	// MGlobal::displayInfo("Input Point number 10 is " + PrintTriple(d->newpoints[10]) + "\n");


	// output geometry data
	MDataHandle hOutput = dataBlock.outputValue(plug);
	hOutput.copy(hInputGeom); // We'll copy everything (cheap operation) and then modify only the points we need. I think?

	MObject outMesh = hInputGeom.asMesh();
	MFnMesh outMeshFn(outMesh);

	// collider vertexlists
	MIntArray pcounts, pconnect;

	// collider handles
	MDataHandle colliderHandle = dataBlock.inputValue(collider);

	// Try block
	MObject colliderObject = colliderHandle.asMesh();
	MFnMesh colliderFn(colliderObject);
	MItMeshVertex colliderIter(colliderHandle.asMesh());
	int polycount = colliderFn.numPolygons();
	colliderFn.getVertices(pcounts, pconnect);
	returnStatus = colliderFn.getPoints(colliderPoints, MSpace::kObject);
	MCheckErr(returnStatus, "Can't get collidermesh. Check connection to deformer node!");


	unsigned int vertexcount = d->newpoints.length();

	MFnMeshData dataCreator;
	MObject newColliderData(dataCreator.create());

	MDataHandle colliderMatrixHandle = dataBlock.inputValue(colliderMatrix);

	// Kind of a pain to convert from a double matrix to float matrix. Is it necessary?
	MFloatMatrix colliderMatrixFloat = colliderMatrixHandle.asFloatMatrix(); 
	float colliderMatrixRaw[4][4];
	colliderMatrixFloat.get(colliderMatrixRaw);
	MMatrix colliderMatrixValue(colliderMatrixRaw);

	// get collider boundingbox for threshold
	MDataHandle colliderBBSizeHandle = dataBlock.inputValue(colliderBBoxSize);
	double3 &colliderBBSizeValue = colliderBBSizeHandle.asDouble3();
	MVector colliderBBVector(colliderBBSizeValue[0], colliderBBSizeValue[1], colliderBBSizeValue[2]);
	double colliderBBSize = colliderBBVector.length();
	double thresholdValue = colliderBBSize * 2;

	MFloatPointArray newColliderPoints;


	// Do the deformation
	if (envelopeValue != 0) {

		// check the offset value
		if (offsetValue != 0) {
			// XXX: Also used Python copy method here
			d->baseColliderPoints.copy(colliderPoints);

			newColliderPoints.clear();
			int nPoints = colliderPoints.length();
			for (int i = 0; i < nPoints; i++) {
				colliderFn.getVertexNormal(i, colliderPointNormal, MSpace::kObject);
				MFloatPoint newColliderPoint(colliderPoints[i].x + colliderPointNormal.x*offsetValue, colliderPoints[i].y + colliderPointNormal.y*offsetValue, colliderPoints[i].z + colliderPointNormal.z*offsetValue );
				newColliderPoints.append(newColliderPoint);
			}

			returnStatus = colliderFn.createInPlace(colliderPoints.length(), polycount, newColliderPoints, pcounts, pconnect);
			MCheckErr(returnStatus, "Can't create offset copy");
		}

		// create a MMeshintersector instance and define neccessary variables
		returnStatus = d->intersector.create(colliderObject, colliderMatrixValue);
		MCheckErr(returnStatus, "Can't create intersector\n");
		d->mmAccelParams = colliderFn.autoUniformGridParams();



		int checkCollision = 0;
		double maxDeformation = 0.0;

		// get deformer weights
		// direct collision deformation:
		unsigned int num_points = d->newpoints.length();
		for (unsigned int k = 0; k < num_points; k++) {
			inMeshFn.getVertexNormal(k, pointNormal, MSpace::kWorld);


			// define an intersection ray from the mesh that should be deformed
			MFloatPoint		raySource(d->newpoints[k].x, d->newpoints[k].y, d->newpoints[k].z);
			MFloatVector	rayDirection(pointNormal);
			MPoint			point(d->newpoints[k]);

			// MeshFn.allIntersections variables
			MIntArray		*faceIds = nullptr;
			MIntArray		*triIds = nullptr;
			bool			idsSorted = true;
			auto			space = MSpace::kWorld;
			double			maxParam = thresholdValue;
			double			tolerance = 1e-9;
			bool			testBothDirs = true;
			auto			accelParams = &d->mmAccelParams;
			bool			sortHits = true;
			MFloatPointArray hitPoints1;
			MFloatArray		hitRayParams(emptyFloatArray);
			MIntArray		*hitFaces = nullptr;
			MIntArray		*hitTriangles = nullptr;
			MFloatArray		*hitBary1 = nullptr;
			MFloatArray		*hitBary2 = nullptr;

			bool gotHit = colliderFn.allIntersections(raySource, rayDirection, faceIds, triIds, idsSorted, space, maxParam, testBothDirs,
				accelParams, sortHits, hitPoints1, &hitRayParams, hitFaces, hitTriangles, hitBary1, hitBary2, tolerance, &returnStatus);
			MCheckErr(returnStatus, "Error during intersection compute!\n");

			if (gotHit == true) {
				// need this to check if collider is in range for collision, because gotHit may also be true if the collider lies half way outside the maxParam

				unsigned int hitCount = hitPoints1.length();
				int signChange = -1000;

				// Check to see if the ray intersects the collider an odd number of times
				for (unsigned int i = 0; i < hitCount - 1; i++) {
					if (hitRayParams[i] * hitRayParams[i + 1] < 0) {
						signChange = i;
						break;
					}
				}

				int collision = 0;

				// Interesting algorithm you got there...
				if ((hitCount == 2) && (signChange + 1 == 1) && (signChange != -1000))
					collision = 1;
				else if ((hitCount > 2) && (hitCount / (signChange + 1) != 2) && (signChange != -1000))
					collision = 1;


				// if the ray intersects the collider mesh an odd number of times and the collider is in range, collision is happening
				if (collision == 1) {

					checkCollision = checkCollision + 1;

					// add this point to the collision array
					deformedPointsIndices.push_back(k);

					// get the closest point on the collider mesh
					d->intersector.getClosestPoint(point, pointInfo);

					MPoint closePoint(pointInfo.getPoint());
					MFloatVector closePointNormal(pointInfo.getNormal());
					MVector offsetVector(closePointNormal.x, closePointNormal.y, closePointNormal.z);

					// normal angle check for backface culling, if the angle is bigger then 90 the face lies on the opposite side of the collider mesh
					float angle = closePointNormal*rayDirection;

					MPoint worldPoint;
					if ((angle > 0) && (backfaceValue == 1)) {
						// ignore the backfaces, reset the point position
						worldPoint = hitPoints1[signChange];
					}
					else {
						worldPoint = closePoint;
						worldPoint = worldPoint * colliderMatrixValue;
					}

					// update the maximum deformation distance for the bulge strength
					double deformationDistance = point.distanceTo(worldPoint);
					if (maxDeformation < deformationDistance)
						maxDeformation = deformationDistance;

					float weight = weightValue(dataBlock, multiIndex, k);
					d->newpoints[k] += (MFloatPoint(worldPoint) - inPoints[k]) * envelopeValue * weight;
				}
			}
		}


		// indirect collision deformation:
		if (checkCollision != 0) {
			for (unsigned int i = 0; i < num_points; i++) {
				inMeshFn.getVertexNormal(i, inMeshNormal, MSpace::kWorld);

				// Isn't it inefficient to do the raycast and then call getClosestPoint after that??
				MPoint indirPoint(d->newpoints[i]);
				d->intersector.getClosestPoint(indirPoint, indirPointInfo);
				MPoint indirClosePoint(indirPointInfo.getPoint());
				MPoint indirWorldPoint = indirClosePoint * colliderMatrixValue;
				double bulgePntsDist = indirPoint.distanceTo(indirWorldPoint);

				float weight = weightValue(dataBlock, multiIndex, i);


				// calculate the relative distance of the meshpoint based on the maximum bulgerange
				double relativedistance = bulgePntsDist / (bulgeExtendValue + 0.00001);

				// get the bulge curve. 
				// XXX: The Python was a bit jank so check on this. Also the UI code MEL might have something to do with it?
				float bulgeAmount = -1;
				bulgeshapeHandle.getValueAtPosition(float(relativedistance), bulgeAmount);

				// set the point position for indirect collision deformation
				d->newpoints[i] += inMeshNormal *bulgeExtendValue*(bulgeValue / 5)*envelopeValue*bulgeAmount*maxDeformation*weight;
			}

			outMeshFn.setPoints(d->newpoints, MSpace::kWorld);
			dataBlock.setClean(outputGeom);

			if (offsetValue != 0) {
				returnStatus = colliderFn.createInPlace(colliderPoints.length(), polycount, d->baseColliderPoints, pcounts, pconnect);
				MCheckErr(returnStatus, "Can't reset offset copy");
			}
		} // Collision check
	} // Do deformation

	return MS::kSuccess;
}

void* ContactDeform::creator()
//
//	Description:
//		this method exists to give Maya a way to create new objects
//      of this type. 
//
//	Return Value:
//		a new object of this type
//
{
	return new ContactDeform();
}

MStatus ContactDeform::initialize()
//
//	Description:
//		This method is called to create and initialize all of the attributes
//      and attribute dependencies for this node type.  This is only called 
//		once when the node type is registered with Maya.
//
//	Return Values:
//		MS::kSuccess
//		MS::kFailure
//		
{

	// Maybe track the status? CHECK_MSTATUS macro does a decent job though
	MStatus status;

	// Start adding attributes
	MFnGenericAttribute gAttr;
	collider = gAttr.create("collider", "coll", &status);
	status = gAttr.addDataAccept(MFnData::kMesh);
	status = gAttr.setHidden(true);
	if (!status) { status.perror("createAttribute"); return status; }

	// Numeric attributes
	MFnNumericAttribute nAttr;
	bulgeextend = nAttr.create("bulgeextend", "bex", MFnNumericData::kDouble, 0.0, &status);
	nAttr.setKeyable(true); // Attribute is keyable and will show up in the channel box
	nAttr.setStorable(true); // Attribute will be written to files when this type of node is stored
	nAttr.setSoftMin(0);  // Range from 0-10
	nAttr.setSoftMax(10);

	bulge = nAttr.create("bulge", "blg", MFnNumericData::kDouble, 1.0, &status);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	nAttr.setSoftMin(0);
	nAttr.setSoftMax(10);

	offset = nAttr.create("offset", "off", MFnNumericData::kDouble, 0.0, &status);
	nAttr.setKeyable(true);
	nAttr.setStorable(true);
	nAttr.setSoftMin(0);
	nAttr.setSoftMax(1);

	colliderBBoxX = nAttr.create("colliderBBoxX", "cbbX", MFnNumericData::kDouble, 0.0);
	colliderBBoxY = nAttr.create("colliderBBoxY", "cbbY", MFnNumericData::kDouble, 0.0);
	colliderBBoxZ = nAttr.create("colliderBBoxZ", "cbbZ", MFnNumericData::kDouble, 0.0);

	MFnCompoundAttribute cAttr;
	colliderBBoxSize = cAttr.create("colliderBBoxSize", "cbb");
	cAttr.addChild(colliderBBoxX);
	cAttr.addChild(colliderBBoxY);
	cAttr.addChild(colliderBBoxZ);

	MFnMatrixAttribute mAttr;
	colliderMatrix = mAttr.create("colliderMatrix", "collMatr", MFnMatrixAttribute::kFloat);
	mAttr.setHidden(true);

	MRampAttribute rAttr;
	bulgeshape = rAttr.createCurveRamp("bulgeshape", "blgshp");

	MFnEnumAttribute eAttr;
	backface = eAttr.create("backface_culling", "bkcul", 0);
	eAttr.addField("off", 0);
	eAttr.addField("on", 1);
	eAttr.setHidden(false);
	eAttr.setKeyable(true);
	eAttr.setStorable(true);

	sculptmode = eAttr.create("sculpt_mode", "snmd", 0);
	eAttr.addField("off", 0);
	eAttr.addField("on", 1);
	eAttr.setHidden(false);
	eAttr.setKeyable(true);
	eAttr.setStorable(true);


	// Add the attributes we have created to the node
	CHECK_MSTATUS(addAttribute(collider));
	CHECK_MSTATUS(addAttribute(bulge));
	CHECK_MSTATUS(addAttribute(bulgeextend));
	CHECK_MSTATUS(addAttribute(colliderMatrix));
	CHECK_MSTATUS(addAttribute(backface));
	CHECK_MSTATUS(addAttribute(sculptmode));
	CHECK_MSTATUS(addAttribute(bulgeshape));
	CHECK_MSTATUS(addAttribute(offset));
	CHECK_MSTATUS(addAttribute(colliderBBoxSize));

	// Set up a dependency between the input and the output.  This will cause
	// the output to be marked dirty when the input changes.  The output will
	// then be recomputed the next time the value of the output is requested.
	auto outputGeom = MPxGeometryFilter::outputGeom;
	CHECK_MSTATUS(attributeAffects(collider, outputGeom));
	CHECK_MSTATUS(attributeAffects(offset, outputGeom));
	CHECK_MSTATUS(attributeAffects(colliderBBoxSize, outputGeom));
	CHECK_MSTATUS(attributeAffects(bulge, outputGeom));
	CHECK_MSTATUS(attributeAffects(bulgeextend, outputGeom));
	CHECK_MSTATUS(attributeAffects(colliderMatrix, outputGeom));
	CHECK_MSTATUS(attributeAffects(backface, outputGeom));
	CHECK_MSTATUS(attributeAffects(sculptmode, outputGeom));
	CHECK_MSTATUS(attributeAffects(bulgeshape, outputGeom));

	if (!status) { status.perror("attributeAffects"); return status; }
	return MS::kSuccess;
}




// accessoryNodeSetup used to initialize the ramp attributes
MStatus ContactDeform::accessoryNodeSetup(MDagModifier &cmd) {
		MRampAttribute bulgeshapeHandle(this->thisMObject(), bulgeshape);
        
		MFloatArray a1;
		MFloatArray b1;
		MIntArray c1;
        
		a1.append(float(0.0));
		a1.append(float(0.2));
		a1.append(float(1.0));
        
        b1.append(float(0.0));
        b1.append(float(1.0));
        b1.append(float(0.0));
        
        c1.append(MRampAttribute::kSpline);
        c1.append(MRampAttribute::kSpline);
		c1.append(MRampAttribute::kSpline);
        
		bulgeshapeHandle.addEntries(a1, b1, c1);
		return MStatus::kSuccess;
}