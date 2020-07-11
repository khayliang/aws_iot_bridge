#!/home/kl/torch_gpu_ros/bin/python

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

    

