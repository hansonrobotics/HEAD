from functools import wraps
from flask import request, Response
import json
json_encode = json.JSONEncoder().encode

def check_auth(auth):
    return auth == 'AAAAB3NzaC'

def authenticate():
    return Response(json_encode({'ret': 401, 'response': {'text': 'Could not verify your access'}}),
        mimetype="application/json")

def requires_auth(f):
    @wraps(f)
    def decorated(*args, **kwargs):
        auth = request.args.get('Auth')
        if not auth or not check_auth(auth):
            return authenticate()
        return f(*args, **kwargs)
    return decorated
