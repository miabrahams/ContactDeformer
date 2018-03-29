//
// Copyright (C) 
// 
// File: pluginMain.cpp
//
// Author: Maya Plug-in Wizard 2.0
//

#include "ContactDeformNode.h"

#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>

MStatus initializePlugin( MObject obj )
//
//	Description:
//		this method is called when the plug-in is loaded into Maya.  It 
//		registers all of the services that this plug-in provides with 
//		Maya.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{ 
	MStatus   status;
	MFnPlugin plugin( obj, "Michael G. Abrahams", "2018", "0.0.1");

	status = plugin.registerNode( "ContactDeform", ContactDeform::id, ContactDeform::creator,
								  ContactDeform::initialize, MPxNode::kDeformerNode );
	if (!status) {
		status.perror("registerNode");
		return status;
	}
	MGlobal::displayInfo("Hello!");

	return status;
}

MStatus uninitializePlugin( MObject obj)
//
//	Description:
//		this method is called when the plug-in is unloaded from Maya. It 
//		deregisters all of the services that it was providing.
//
//	Arguments:
//		obj - a handle to the plug-in object (use MFnPlugin to access it)
//
{
	MStatus   status;
	MFnPlugin plugin( obj );

	status = plugin.deregisterNode( ContactDeform::id );
	if (!status) {
		status.perror("deregisterNode");
		return status;
	}

	return status;
}
