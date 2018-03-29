#ifndef _ContactDeformNode
#define _ContactDeformNode
//
// Copyright (C) 2018 Michael Abrahams
// 
// File: ContactDeformNode.h
//
// Dependency Graph Node: ContactDeform
//
// Author: Michael Abrahams
//         miabraha@gmail.com
//

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MTypeId.h> 
#include <maya/MPxDeformerNode.h>

struct ContactDeformPrivate;
 
class ContactDeform : public MPxDeformerNode
{
public:
						ContactDeform();
	virtual				~ContactDeform(); 

	virtual MStatus		compute( const MPlug& plug, MDataBlock& data );

	static  void*		creator();
	static  MStatus		initialize();
	MStatus				accessoryNodeSetup(MDagModifier &cmd);

public:

	// There needs to be a MObject handle declared for each attribute that
	// the node will have.  These handles are needed for getting and setting
	// the values later.
	//
	static MObject collider;
	static MObject bulgeextend;
	static MObject bulge;
	static MObject offset;
	static MObject colliderBBoxX;
	static MObject colliderBBoxY;
	static MObject colliderBBoxZ;
	static MObject colliderBBoxSize;
	static MObject colliderMatrix;
	static MObject bulgeshape;
	static MObject backface;
	static MObject sculptmode;

	// The typeid is a unique 32bit indentifier that describes this node.
	// It is used to save and retrieve nodes of this type from the binary
	// file format.  If it is not unique, it will cause file IO problems.
	//
	static	MTypeId		id;

private: 
	ContactDeformPrivate *d;
	int maxdist = 0;

    
};

#endif
