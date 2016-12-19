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
- *test* - If true, start a test session, else start a normal session
- *refresh* - If true, start a new session, else try to reuse the session binding with the same user name

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
- *query* - It's a try ask (True or False)

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

Parameters: 
- *Auth* - Authorization token

Return:
- *ret* - Return code
- *response*

### Set Weights for Each Tier in the Chain

```
GET /v1.1/set_weights
```

Parameters:
- *Auth* - Authorization token
- *param* - Tier weight parameter. Could be, for example, "0=1, 1=0.5", or "reset"
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
- *index* - Index of the answer in the session. For exmaple, -1 means the last response.
- *rate* - Rate string, such as "good" or "bad".

Return:
- *ret* - Return code
- *response*

### List current sessions

```
GET /v1.1/sessions
```

Parameters:
- *Auth* - Authorization token

Return:
- *ret* - Return code
- *response*

### Set custom context

```
GET /v1.1/set_context
```

Parameters:
- *Auth* - Authorization token
- *session* - Session ID
- *context* - Context string (format key=value,key2=value2)

Return:
- *ret* - Return code
- *response*

### Remove context

```
GET /v1.1/remove_context
```

Parameters:
- *Auth* - Authorization token
- *session* - Session ID
- *context* - Context keys (format key,key2,key3,...)

Return:
- *ret* - Return code
- *response*

### Get session context

```
GET /v1.1/get_context
```

Parameters:
- *Auth* - Authorization token
- *session* - Session ID

Return:
- *ret* - Return code
- *response*

### Add the message to the output history of the control tier, as the tier has said that before

```
GET /v1.1/said
```

Parameters:
- *Auth* - Authorization token
- *session* - Session ID
- *message* - Message text

Return:
- *ret* - Return code
- *response*
