from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient, AWSIoTMQTTShadowClient
import rospy
import logging

__all__ = ['IOTCoreClient']

class IOTCoreClient(AWSIoTMQTTClient, AWSIoTMQTTShadowClient):
    def __init__(self, verbosity):
        aws_config = rospy.get_param("/aws_config")

        # Configure logging
        logger = logging.getLogger("AWSIoTPythonSDK.core")
        if verbosity:
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.WARNING)

        streamHandler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        streamHandler.setFormatter(formatter)
        logger.addHandler(streamHandler)

        super().__init__(aws_config["client_id"])
        super().configureEndpoint(aws_config["host"], aws_config["port"])

        super().configureCredentials(aws_config["root_cert"],\
        aws_config["private_key"], aws_config["cert"])
        
        super().configureConnectDisconnectTimeout(10)  # 10 sec
        super().configureMQTTOperationTimeout(10)  # 5 sec

        if super().connect():
            rospy.loginfo("Connected to device")
        else:
            rospy.logerr("Couldn't connect to device")
