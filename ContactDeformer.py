# Copyright (C) 2018 Michael Abrahams
# Python function to add the contact deformer to two selected objects.

import pymel.core as pm

def ContactDeformer():
    sel = pm.ls(sl=True)
    if len(sel) is 2:
        collider = sel[0]
        target = sel[1]
        collidershape = pm.listRelatives(collider, s=True)[0]
        collisiondeformer = pm.deformer(target, type="ContactDeform", n="collisionDeformer")[0]
        pm.connectAttr(collidershape.worldMesh[0], collisiondeformer.collider, f=True)
        pm.connectAttr(collider.matrix, collisiondeformer.colliderMatrix, f=True)
        pm.connectAttr(collider.boundingBoxSize, collisiondeformer.colliderBBoxSize, f=True)
    else:
        print("Please select two meshes: first the collider mesh then the mesh that should be deformed.")

ContactDeformer()
