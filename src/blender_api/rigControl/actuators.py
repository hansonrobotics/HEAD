import inspect
from collections import namedtuple
from time import time as systime

try:
    import bpy
except ImportError:
    pass

_context = None

def sleep(delay):
    end = _context.time + delay
    yield
    while _context.time < end:
        yield


actuator_functions = []
def new(func):
    actuator_functions.append(func)
    return func


class ActuatorManager:

    class Actuator:

        class Parameter:
            def __init__(self, getter, setter):
                self.getter = getter
                self.setter = setter
            @property
            def val(self):
                return self.getter()
            @val.setter
            def val(self, val):
                self.setter(val)

        class Image:
            def __init__(self, pixels_setter):
                self.set_pixels = pixels_setter

        def __init__(self, name, register_parameter, register_image):
            self.name = name
            self.register_parameter = register_parameter
            self.register_image = register_image
            self.generator = None

        def add_parameter(self, bl_prop):
            getter, setter = self.register_parameter(self.name, bl_prop)
            return self.Parameter(getter, setter)

        def add_image(self, image_name, img_size):
            pixels_setter = self.register_image(self.name, image_name, img_size)
            return self.Image(pixels_setter)

    Context = namedtuple('Context', ['time', 'dt'])

    def __init__(self):
        self.property_store = PropertyStore()

        import artistic.actuators

        self.actuators = []
        with _set_global('_context', self.Context(systime(), 0)):
            for func in actuator_functions:
                self.property_store.register_actuator(func.__name__)
                actuator = self.Actuator(func.__name__,
                    self.property_store.register_parameter,
                    self.property_store.register_image)
                actuator.generator = func(actuator) if _takes_nargs(func, 1) else func()
                self.actuators.append(actuator)
                actuator.generator.send(None)

    def tick(self, time, dt):
        with _set_global('_context', self.Context(time, dt)):
            for actuator in self.actuators:
                actuator.generator.send((time, dt))


class PropertyStore:

    def __init__(self):
        self.bltype_actuators = type("HR_Actuators", (bpy.types.PropertyGroup,), {})
        bpy.utils.register_class(self.bltype_actuators)
        bpy.types.Scene.actuators = bpy.props.PointerProperty(type=self.bltype_actuators)

    def register_actuator(self, actuator_name):
        bltype_act = type('HR_ACT_{}'.format(actuator_name.capitalize()), (bpy.types.PropertyGroup,), {})
        bpy.utils.register_class(bltype_act)
        setattr(self.bltype_actuators, 'ACT_'+actuator_name, bpy.props.PointerProperty(type=bltype_act))

    def register_parameter(self, actuator_name, bl_prop):
        bltype_act = getattr(bpy.types, 'HR_ACT_{}'.format(actuator_name.capitalize()))
        prop_name = 'PARAM_{}'.format(bl_prop[1]['name'].replace(' ', '_'))
        setattr(bltype_act, prop_name, bl_prop)

        def getter():
            bl_act = getattr(bpy.context.scene.actuators, 'ACT_'+actuator_name)
            return getattr(bl_act, prop_name)
        def setter(val):
            bl_act = getattr(bpy.context.scene.actuators, 'ACT_'+actuator_name)
            return setattr(bl_act, prop_name, val)

        return getter, setter

    def register_image(self, actuator_name, image_name, img_size):
        bltype_act = getattr(bpy.types, 'HR_ACT_{}'.format(actuator_name.capitalize()))
        prop_name = 'IMG_{}'.format(image_name.replace(' ', '_'))
        texture_name = 'HR_ACT_{}_{}'.format(actuator_name.replace(' ', '_'), image_name.replace(' ', '_'))
        setattr(bltype_act, prop_name, bpy.props.StringProperty(default=texture_name))

        texture = bpy.data.textures.get(texture_name) or bpy.data.textures.new(texture_name, 'IMAGE')
        texture.extension = 'CLIP'
        image = bpy.data.images.get(texture_name) or bpy.data.images.new(texture_name, *img_size)
        texture.image = image

        def set_pixels(pixels):
            image = bpy.data.images[texture_name]
            image.pixels = pixels

        return set_pixels


def _takes_nargs(func, n):
    spec = inspect.getargspec(func)
    return len(spec.args) >= n or spec.varargs != None


def _set_global(name, value):
    class SetGlobal:
        def __enter__(self):
            globals()[name] = value
        def __exit__(self, type, val, tb):
            globals()[name] = None
    return SetGlobal()
