import roslibpy
import rospy
import json
from enum import Enum

class BridgeTypes(Enum):
    SUBSCRIBER = 1
    PUBLISHER = 2
    SERVICE = 3

def bridge_factory(topic, msg_type, iot_client, ros_client, bridge_type):
    if bridge_type.name is "SUBSCRIBER":
        callback = SubscriberBridge(topic, msg_type, iot_client, ros_client)
        listener = roslibpy.Topic(ros_client, topic, msg_type)
        listener.subscribe(callback) 
    if bridge_type.name is "PUBLISHER":
        callback = PublisherBridge(topic, msg_type, iot_client, ros_client)
        iot_client.subscribe(topic, 1, callback)
    if bridge_type.name is "SERVICE":
        callback = ServiceBridge(topic, msg_type, iot_client, ros_client)  
        iot_client.subscribe(topic, 1, callback)

class BaseBridge(object):
    def __init__(self, topic, msg_type, iot_client, ros_client):
        self.topic = topic
        self.type = msg_type
        self.iot_client = iot_client
        self.ros_client= ros_client

class SubscriberBridge(BaseBridge):
    def __call__(self, message):
        message_json = json.dumps(message)
        self.iot_client.publish(self.topic, message_json, 1)

class PublisherBridge(BaseBridge):
    def __init__(self, topic, msg_type, iot_client, ros_client):
        super().__init__(topic, msg_type, iot_client, ros_client)
        self.talker = roslibpy.Topic(ros_client, topic, msg_type)

    def __call__(self, client, userdata, message):
        message_json = json.loads(message.payload)
        self.talker.publish(roslibpy.Message(message_json)) 

class ServiceBridge(BaseBridge):
    def __init__(self, topic, srv_type, iot_client, ros_client):
        super().__init__(topic, srv_type, iot_client, ros_client)
        self.service = roslibpy.Service(ros_client, topic, srv_type)
    def __call__(self, client, userdata, message):

        def callback(message):
            result_json = json.dumps(message.data)
            self.iot_client.publish(self.topic + "/result", result_json, 1)

        message_json = json.loads(message.payload)
        rospy.logdebug("Received: " + json.dumps(message_json) + " from AWS")
        request = roslibpy.ServiceRequest(message_json)
        result = self.service.call(request, callback)
