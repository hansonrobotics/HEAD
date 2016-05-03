This is a premature question-answering server. The goal is to provide a ROS independent, scalable, platform-independent QA.

## Run Server
`$ python run.py [port]`

The default port is 8001.

## Load Characters
The default path of the characters is the current [characters](https://github.com/hansonrobotics/public_ws/tree/master/src/chatbot/src/server/characters) directory.
But you can overwrite it by setting the environment variable `HR_CHARACTER_PATH`. For example

`$ HR_CHARACTER_PATH=/path/to/characters1:/path/to/characters2 python run.py`

### How is the Character Loaded
It loads every python module except `__init__.py` in every character path in `HR_CHARACTER_PATH`, and finds the global `characters` in this module.
Then it appends the characters to its character list `CHARACTERS`. See [`characters/__init__.py`](https://github.com/hansonrobotics/public_ws/blob/master/src/chatbot/src/server/characters/__init__.py)

## Define a Character
### Define an AIML character

```python
from server.character import AIMLCharacter
# id is an unique global string that refers to this character. name is the character name.
character = AIMLCharacter(id, name)
character.load_aiml_files([file1, file2])
character.set_property_file(abc.properties) # Set the key,value properties.
```

Once it's done, you need to put this object to global variable `characters` so the server can find and load it.

See the example in [characters/aiml_characters.py#Line42](https://github.com/hansonrobotics/public_ws/blob/master/src/chatbot/src/server/characters/aiml_characters.py#L42)

There is a convienent way to load the definition from yaml file.

See [characters/aiml_characters.py#Line44](https://github.com/hansonrobotics/public_ws/blob/master/src/chatbot/src/server/characters/aiml_characters.py#L44)

The yaml file format is like this. The path is relative to the yaml file itself.
```yaml
id:
    quant_han
name:
    han
property_file:
    ../../../character_aiml/han.properties
aiml:
    - ../../../futurist_aiml/agians.aiml
    - ../../../futurist_aiml/aiexistsans.aiml
    - ...
```
### Define a third party character
For example, if you want to integrate IBM Waston chatbot. What you can do is

1. Create a sub-class that inherits [Character](https://github.com/hansonrobotics/public_ws/blob/master/src/chatbot/src/server/character.py#L7) class.
2. Implement `respond` method. The return value is a dict that has these keys `response`, `botid`, `botname`.

Here is a dummy example.
```python
from server.character import Character
class DummyCharacter(Character):
  def respond(self, question, session=None):
    ret['response'] = "Hi there"
    ret['botid'] = self.id
    ret['botname'] = self.name
    return ret

characters = [DummyCharacter('34g634', 'dummy')]
```
Then you can put this module to the character path so when the server is started, the character can be found and loaded.

## Client for testing
There is a client for testing purpose. It's [client.py](https://github.com/hansonrobotics/public_ws/blob/master/src/chatbot/src/server/client.py).

Run `$ python client.py`, then you can ask questions and get the answers.

```
$ python client.py
[me]: help

Documented commands (type help <topic>):
========================================
chatbot  conn  help  list  q

[me]: list
[han]
sophia
pkd
quant_han
generic
[me]: chatbot quant_han
Set chatbot to quant_han
[me]: can ai make money
quant_han[by han]: Making billions is all good fun. But obsoleting the money economy is what it's all about.
```
