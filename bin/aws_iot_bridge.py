#!/home/kl/torch_gpu_ros/bin/python

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from iot_core import IOTCoreClient
import bridges

import roslibpy
import rospy
import argparse
import json
import time
import signal
import logging

class IOTBridge(object): 
    def __init__(self, args):
        self.bridge_params = rospy.get_param("/aws_iot_bridge")

        self.device = IOTCoreClient(args.verbose)

        self.ros = roslibpy.Ros(host="localhost", port=9090)
        self.ros.run()

        def check_if_connected():
            if self.ros.is_connected:
                rospy.loginfo("Connected to ROSBridge websocket.")
            else:
                rospy.logerr("Unable to connect to ROSBridge websocket. Check if you launched the ROSBridge node")
                
        self.ros.on_ready(check_if_connected)

        for subscriber in self.bridge_params["subscribers"]:
            bridges.bridge_factory(subscriber["topic"], subscriber["type"],\
                self.device, self.ros, bridges.BridgeTypes.SUBSCRIBER)

        for publisher in self.bridge_params["publishers"]:
            bridges.bridge_factory(publisher["topic"], publisher["type"],\
                self.device, self.ros, bridges.BridgeTypes.PUBLISHER)

        for service in self.bridge_params["services"]:
            bridges.bridge_factory(service["topic"], service["type"],\
                self.device, self.ros, bridges.BridgeTypes.SERVICE)

    def terminate(self):
        rospy.loginfo("Terminating ROSBridge and IOT Core connection")
        self.ros.terminate()
        self.device.disconnect()

    def init_sub_bridge(self, topic, msg_type):

        # Creating a nested class is necessary since device.publish is a function
        outer_class_self = self
        class SubscriberCallback(object):
            def __init__(self, topic):
                self.topic = topic
            def __call__(self, message):
                message_json = json.dumps(message)
                outer_class_self.device.publish(self.topic, message_json, 1)

        callback = SubscriberCallback(topic)

        listener = roslibpy.Topic(self.ros, topic, msg_type)
        listener.subscribe(callback)   

    def init_pub_bridge(self, topic, msg_type):

        outer_class_self = self
        class PublisherCallback(object):
            def __init__(self, topic, msg_type):
                self.topic = topic
                self.talker = roslibpy.Topic(outer_class_self.ros, topic, msg_type)

            def __call__(self, client, userdata, message):
                message_json = json.loads(message.payload)
                self.talker.publish(roslibpy.Message(message_json)) 

        callback = PublisherCallback(topic, msg_type)  

        self.device.subscribe(topic, 1, callback)

    def init_srv_bridge(self, topic, srv_type):

        outer_class_self = self
        class ServiceCall(object):
            def __init__(self, topic, srv_type):
                self.topic = topic
                self.service = roslibpy.Service(outer_class_self.ros, topic, srv_type)
            def __call__(self, client, userdata, message):
                message_json = json.loads(message.payload)
                request = roslibpy.ServiceRequest(message_json)
                result = self.service.call(request)
                print(result.data)
                result_json = json.dumps(result.data)
                try:
                    outer_class_self.device.publish(self.topic + "/result", result_json, 1)
                except:
                    rospy.logerr("AWS time out")

        callback = ServiceCall(topic, srv_type)  

        self.device.subscribe(topic, 1, callback)



def parse_args():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true")

    return parser.parse_known_args()
            
def main(args):
    '''Initializes and cleanup ros node'''
    bridge = IOTBridge(args)
    rospy.init_node('aws_iot_bridge', anonymous=True)

    while not rospy.is_shutdown():
        rospy.spin()

    rospy.loginfo("Killing IOT Bridge")
    bridge.terminate()



if __name__=="__main__":
    args, unknown = parse_args()
    main(args)

    

