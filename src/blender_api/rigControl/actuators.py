from time import time

class Parameter:
    def __init__(self, name, min=0.0, max=1.0):
        pass


def sleep(delay):
    end = time() + delay
    while time() < end:
        yield


actuator_functions = []
def actuator(func):
    actuator_functions.append(func)
    return func

class ActuatorManager:

    def __init__(self):
        import artistic.actuators

        self.actuator_functions = []
        for func in actuator_functions:
            generator = func()
            generator.send(None)
            self.actuator_functions.append(generator)

    def tick(self, time, dt):
        for anim in self.actuator_functions:
            anim.send((time, dt))
