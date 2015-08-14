# This module defines the API for a source of commands that
# the command listener will listen to.

# Virtual base class for sources of animation commands.
# Any class inheriting from thos one can be used to send commands to
# blender. Use register_cmd_source(), below, to declare a new command
# source to the blender api.
class CommandSource:

    # When called, should return the next command that blender will run.
    def poll(self):
        return None

    def push(self):
        return

    def init(self):
        return False

    # After this is called, blender will never again poll for more
    # commands. This is a great time to empty out any pending queues.
    def drop(self):
        return
