def register():
    import sys
    import bpy
    import imp

    # setup package path
    file_path = bpy.path.abspath('//')
    sys.path.append(file_path)

    # import package and force refresh for dev
    import rigControl
    imp.reload(rigControl)
    rigControl.init()

register()
