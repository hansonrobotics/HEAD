#!/usr/bin/python3

from flask import Flask, send_from_directory
import json, reporter

json_encode = json.JSONEncoder().encode

app = Flask(__name__)
rep = reporter.Reporter('checks.yaml')

@app.route('/system/status')
def send_status():
    return json_encode(rep.report())

@app.route('/')
def send_index():
    return send_public('index.html')

@app.route('/<path:filename>')
def send_public(filename):
    return send_from_directory('public', filename)

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, use_reloader=False)
