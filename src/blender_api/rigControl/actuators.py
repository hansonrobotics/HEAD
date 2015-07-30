import inspect
from collections import namedtuple
from time import time as systime
import bpy

_context = None

def sleep(delay):
    end = _context.time + delay
    yield
    while _context.time < end:
        yield


actuator_functions = []
def actuator(func):
    actuator_functions.append(func)
    return func


class ActuatorManager:

    class Actuator:

        class Parameter:
            def __init__(self, getter):
                self.getter = getter
            @property
            def val(self):
                return self.getter()

        def __init__(self, name, register_parameter):
            self.name = name
            self.register_parameter = register_parameter
            self.generator = None

        def add_parameter(self, bl_prop):
            getter = self.register_parameter(self.name, bl_prop)
            return self.Parameter(getter)

    Context = namedtuple('Context', ['time', 'dt'])

    def __init__(self):
        self.property_store = PropertyStore()

        import artistic.actuators

        self.actuators = []
        with _set_global('_context', self.Context(systime(), 0)):
            for func in actuator_functions:
                self.property_store.register_actuator(func.__name__)
                actuator = self.Actuator(func.__name__, self.property_store.register_parameter)
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
        bltype = type('HR_ACT_{}'.format(actuator_name.capitalize()), (bpy.types.PropertyGroup,), {})
        bpy.utils.register_class(bltype)
        setattr(self.bltype_actuators, 'ACT_'+actuator_name, bpy.props.PointerProperty(type=bltype))

    def register_parameter(self, actuator_name, bl_prop):
        bltype_act = getattr(bpy.types, 'HR_ACT_{}'.format(actuator_name.capitalize()))
        prop_name = 'PARAM_{}'.format(bl_prop[1]['name'].replace(' ', '_'))
        setattr(bltype_act, prop_name, bl_prop)

        def getter():
            bl_act = getattr(bpy.context.scene.actuators, 'ACT_'+actuator_name)
            return getattr(bl_act, prop_name)

        return getter


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
