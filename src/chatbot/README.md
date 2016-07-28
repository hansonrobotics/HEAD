This is a premature question-answering server. The goal is to provide a ROS independent, scalable, platform-independent QA.

## Run Server
`$ python run.py [port]`

The default port is 8001.

## Load Characters
The default path of the characters is the current [characters](https://github.com/hansonrobotics/HEAD/tree/master/src/chatbot/scripts/characters) directory.
But you can overwrite it by setting the environment variable `HR_CHARACTER_PATH`. Use comma seperator. For example

`$ HR_CHARACTER_PATH=/path/to/characters1:/path/to/characters2 python run.py`

## Define a Character

You can wirte either Python or YAML to define your character.

### Write Python module
```python
from server.character import AIMLCharacter
# id is an unique global string that refers to this character. name is the character name.
character = AIMLCharacter(id, name)
character.load_aiml_files([file1, file2])
character.set_property_file(abc.properties) # Set the key,value properties.

characters = [character] # you need to put character to this module-wide variable so the server can find and load it
```

### Write YAML file

There is a convienent way to load the definition from yaml file.

The yaml file format is like this. The path is relative to the yaml file itself.
```yaml
id:
    han
name:
    han
level:
    90
weight:
    1
property_file:
    ../../../character_aiml/han.properties
aiml:
    - ../../../futurist_aiml/agians.aiml
    - ../../../futurist_aiml/aiexistsans.aiml
    - ...
```

## Client for testing
There is a client for testing purpose. It is called [client.py](https://github.com/hansonrobotics/HEAD/blob/master/src/chatbot/scripts/client.py).

Run `$ python client.py`, then you can ask questions and get the answers. Type `help` for the usage.

## Chatbot Server API (v1.1)

### Start a session

```
GET /v1.1/start_session
```

Parameters: 
- *Auth* - Authorization token
- *botname* - Chatbot name
- *user* - User name

Return:
- *ret* - Return code
- *sid* - Session ID

### List responding character chain

```
GET /v1.1/chatbots
```

Parameters: 
- *Auth* - Authorization token
- *lang* - Language
- *session* - Session ID

Return:
- *ret* - Return code
- *response*

### Chat

```
GET /v1.1/chat
```

Parameters: 
- *Auth* - Authorization token
- *question* - Question
- *lang* - Language
- *session* - Session ID

Return:
- *ret* - Return code
- *response*

### Batch Chat

```
POST /v1.1/batch_chat
```

Parameters: 
- *Auth* - Authorization token
- *questions* - Questions
- *lang* - Language
- *session* - Session ID

Return:
- *ret* - Return code
- *response* - List of responses

### Get Bot Names

```
GET /v1.1/bot_names
```

Return:
- *ret* - Return code
- *response*

### Set Weights for Each Tier in the Chain

```
GET /v1.1/set_weights
```

Parameters:
- *Auth* - Authorization token
- *weights* - Tier weights
- *session* - Session ID

Return:
- *ret* - Return code
- *response*

### Upload character

```
GET  /v1.1/upload_character
```

Parameters:
- *Auth* - Authorization token
- *user* - User name
- *zipfile* - Character package

Return:
- *ret* - Return code
- *response*

### Rate the answer

```
GET /v1.1/rate
```

Parameters:
- *Auth* - Authorization token
- *session* - Session ID
- *index* - Index of the answer in the session
- *rate* - Rate string

Return:
- *ret* - Return code
- *response*
