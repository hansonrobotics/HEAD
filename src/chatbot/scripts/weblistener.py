#!/usr/bin/env python

from chatbot.msg import ChatMessage
from flask import Flask
from flask import render_template
import rospy

app = Flask(__name__)
app.config['WS_HOST'] = 'localhost'

@app.route("/")
def home():
  return render_template('index.html', ws_host=app.config['WS_HOST'])

class ChatbotWebSpeechRecognizer:
  def __init__(self):
    rospy.init_node('chatbot_web_listener')

  def start(self):
    app.run(host='0.0.0.0')

def main():
  recognizer = ChatbotWebSpeechRecognizer()
  recognizer.start() 

if __name__ == '__main__':
  main()
