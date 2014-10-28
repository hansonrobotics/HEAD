import sys
import bpy
import imp

# setup package path
filePath = bpy.path.abspath('//')
sys.path.append(filePath)

# import package and force refresh for dev
import rigControl
imp.reload(rigControl)