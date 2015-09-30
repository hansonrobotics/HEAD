import inspect
from collections import namedtuple
from time import time as systime

try:
    import bpy
except ImportError:
    pass

# Global variable that contains current time during the execution of an
# actuator step. Useful for the sleep method below.
_context = None

def sleep(delay):
    """ A non-thread-blocking sleep function to be used from inside the
    actuator function. """
    end = _context.time + delay
    yield
    while _context.time < end:
        yield


actuator_functions = []
def new(func):
    """ Decorator that registers actuator functions with the manager. """
    actuator_functions.append(func)
    return func


class ActuatorManager:

    class Actuator:
        """ An interface between the actuator functions and the actuator manager.
        Actuator functions are passed an instance of this class as 'self'. """

        class Parameter:
            """ An interface returned by add_parameter. """
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
            """ An interface returned by add_image. """
            def __init__(self, pixels_setter):
                self.set_pixels = pixels_setter

        def __init__(self, name, register_parameter, register_image, is_enabled):
            self.name = name
            self.register_parameter = register_parameter
            self.register_image = register_image
            self.is_enabled = is_enabled
            self.generator = None

        def add_parameter(self, bl_prop):
            """ Register a new variable parameter with the actuator manager.
            Should be called in the beginning of the artistic actuator function.
            bl_prop - (a Blender property instance of type bpy.props.*)
            """
            getter, setter = self.register_parameter(self.name, bl_prop)
            return self.Parameter(getter, setter)

        def add_image(self, image_name, img_size):
            """ Register a new image with the actuator manager.
            Should be called in the beginning of the artistic actuator function.
            image_name - (string)
            image_size - (tuple of 2 ints)
            """
            pixels_setter = self.register_image(self.name, image_name, img_size)
            return self.Image(pixels_setter)

    Context = namedtuple('Context', ['time', 'dt'])

    def __init__(self):
        self.property_store = PropertyStore()

        # Initialize artistic actuator functions.
        import artistic.actuators
        self.actuators = []
        with _set_global('_context', self.Context(systime(), 0)):
            for func in actuator_functions:
                enabled_getter = self.property_store.register_actuator(func.__name__)
                actuator = self.Actuator(func.__name__,
                    self.property_store.register_parameter,
                    self.property_store.register_image,
                    enabled_getter)
                actuator.generator = func(actuator) if _takes_nargs(func, 1) else func()
                self.actuators.append(actuator)
                actuator.generator.send(None)

    def tick(self, time, dt):
        """ Called every frame from blenderPlayback.py """
        with _set_global('_context', self.Context(time, dt)):
            for actuator in self.actuators:
                if actuator.is_enabled():
                    actuator.generator.send((time, dt))


class PropertyStore:
    """ Manage actuator properties and data in Blender RNA """

    def __init__(self):
        self.bltype_actuators = type("HR_Actuators", (bpy.types.PropertyGroup,), {})
        bpy.utils.register_class(self.bltype_actuators)
        bpy.types.Scene.actuators = bpy.props.PointerProperty(type=self.bltype_actuators)

    def register_actuator(self, actuator_name):
        # Update the scene.actuators type to include the new actuator.
        bltype_act = type('HR_ACT_{}'.format(actuator_name.capitalize()), (bpy.types.PropertyGroup,), {})
        bltype_act.parameter_order = bpy.props.StringProperty()
        bpy.utils.register_class(bltype_act)
        setattr(self.bltype_actuators, 'ACT_'+actuator_name, bpy.props.PointerProperty(type=bltype_act))

        # Include an on/off boolean in the actuator type.
        setattr(bltype_act, 'HEAD_PARAM_enabled', bpy.props.BoolProperty())

        bl_act = getattr(bpy.context.scene.actuators, 'ACT_'+actuator_name)
        def enabled_getter():
            return bl_act.HEAD_PARAM_enabled
        return enabled_getter

    def register_parameter(self, actuator_name, bl_prop):
        # Update the corresponding actuator's type to include the new parameter.
        bltype_act = getattr(bpy.types, 'HR_ACT_{}'.format(actuator_name.capitalize()))
        prop_name = 'PARAM_{}'.format(bl_prop[1]['name'].replace(' ', '_'))
        setattr(bltype_act, prop_name, bl_prop)

        self._update_order('ACT_'+actuator_name, prop_name)

        bl_act = getattr(bpy.context.scene.actuators, 'ACT_'+actuator_name)
        def getter():
            return getattr(bl_act, prop_name)
        def setter(val):
            return setattr(bl_act, prop_name, val)

        return getter, setter

    def register_image(self, actuator_name, image_name, img_size):
        # Update the corresponding actuator's type to include the new parameter.
        bltype_act = getattr(bpy.types, 'HR_ACT_{}'.format(actuator_name.capitalize()))
        prop_name = 'IMG_{}'.format(image_name.replace(' ', '_'))
        texture_name = 'HR_ACT_{}_{}'.format(actuator_name.replace(' ', '_'), image_name.replace(' ', '_'))
        setattr(bltype_act, prop_name, bpy.props.StringProperty(default=texture_name))

        self._update_order('ACT_'+actuator_name, prop_name)

        # Create new texture and image if they don't yet exist.
        texture = bpy.data.textures.get(texture_name) or bpy.data.textures.new(texture_name, 'IMAGE')
        texture.extension = 'CLIP'
        image = bpy.data.images.get(texture_name) or bpy.data.images.new(texture_name, *img_size)
        texture.image = image

        def set_pixels(pixels):
            image = bpy.data.images[texture_name]
            image.pixels = pixels

        return set_pixels

    def _update_order(self, actuator_name, prop_name):
        """ Remember the order in which parameters were registered by updating
        the semicolon seperated string: 'parameter_order'. The order is required
        for UI generation.
        """
        bl_act = getattr(bpy.context.scene.actuators, actuator_name)
        order = bl_act.parameter_order.split(';')
        if prop_name in order:
            order.remove(prop_name)
        order.append(prop_name)
        bl_act.parameter_order = ';'.join(order)


def _takes_nargs(func, n):
    """ Checks whether the given function takes n arguments either directly or
    via *args. """
    spec = inspect.getargspec(func)
    return len(spec.args) >= n or spec.varargs != None


def _set_global(name, value):
    """ Syntactic sugar for temporarily setting a global variable.
    The following example would set a global variable 'myvar' to 7 before
    some_function is called and unset it afterwards:
    with _set_global('myvar', 7):
        some_function()
    """
    class SetGlobal:
        def __enter__(self):
            globals()[name] = value
        def __exit__(self, type, val, tb):
            globals()[name] = None
    return SetGlobal()
