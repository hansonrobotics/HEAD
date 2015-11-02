import bpy
def shapekeyDup():
    if not bpy.context.object.show_only_shape_key:
        bpy.context.object.show_only_shape_key = True
        
    leftname = bpy.context.object.active_shape_key.name
    leftindex = bpy.data.shape_keys["ShapeKeys"].key_blocks.keys().index(leftname)
    if 'L' in leftname:
        rightname = leftname.replace('L', 'R')
    elif 'R' in leftname:
        rightname = leftname.replace('R', 'L')
    else:
        bpy.context.object.show_only_shape_key = False
        print ('Warning: Skipping duplication due to no L or R in namespace\n')
        return
    rightindex = bpy.data.shape_keys["ShapeKeys"].key_blocks.keys().index(rightname)
        
    dupname = 'DUP_L_TO_R'
    if dupname in bpy.data.shape_keys["ShapeKeys"].key_blocks.keys():
        dupindex = bpy.data.shape_keys["ShapeKeys"].key_blocks.keys().index(dupname)
        bpy.context.object.active_shape_key_index = dupindex
        bpy.ops.object.shape_key_remove(all=False)

    bpy.context.object.active_shape_key_index = leftindex
    bpy.ops.object.shape_key_add(from_mix=True)
    bpy.ops.object.shape_key_mirror(use_topology=False)


    bpy.data.shape_keys["ShapeKeys"].key_blocks[bpy.data.shape_keys["ShapeKeys"].key_blocks.keys()[-1]].name = "DUP_L_TO_R"
    dupindex = bpy.data.shape_keys["ShapeKeys"].key_blocks.keys().index(dupname)
    bpy.context.object.active_shape_key_index = dupindex

    bpy.context.object.active_shape_key_index = rightindex

    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.blend_from_shape(shape='DUP_L_TO_R', blend=1.0, add=False)
    print('Shape key mirrored to other side\n')
        
    bpy.ops.sculpt.sculptmode_toggle()

    if dupname in bpy.data.shape_keys["ShapeKeys"].key_blocks.keys():
        dupindex = bpy.data.shape_keys["ShapeKeys"].key_blocks.keys().index(dupname)
        bpy.context.object.active_shape_key_index = dupindex
        bpy.ops.object.shape_key_remove(all=False)

    bpy.context.object.show_only_shape_key = False
    bpy.context.object.active_shape_key_index = leftindex
    
shapekeyDup()