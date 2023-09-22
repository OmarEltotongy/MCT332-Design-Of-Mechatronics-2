import rospy
from std_msgs.msg import String
from rospy import Subscriber, Publisher
from flask import Flask
import os

os.system("fuser -k 5000/tcp")

cur_status = 'waiting'

def clbk(msg):
    global cur_status
    cur_status = msg.data

rospy.init_node("wifi_server_node", anonymous=True)
sub = Subscriber('/status_topic', String, clbk)
pub = Publisher('/req_topic', String, queue_size=10)

app = Flask(__name__)

@app.route("/")
def home():
    global cur_status
    return cur_status

@app.route("/<request>")
def certain_request(request):
    if request == 'start':
        pub.publish(String('start'))
    else:
        pub.publish(String(request))
    return request

if __name__ == "__main__":
    app.run(host='0.0.0.0')
