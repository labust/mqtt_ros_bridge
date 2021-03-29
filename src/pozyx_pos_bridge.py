#!/usr/bin/python

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import bridge
import rospy
import signal
import json
import math

class pozyx_bridge(bridge.bridge):

    def __init__(self, tag_id, mqtt_topic, client_id = "bridge",user_id = "",password = "", host = "localhost", port = "1883", keepalive = 60, topic = "position"):
	self.setup(tag_id,topic)
        bridge.bridge.__init__(self,mqtt_topic, client_id,user_id,password, host, port, keepalive)

    def setup(self, tag_id, topic):
        self.pozyx_tag = tag_id
        self.pos_publisher = rospy.Publisher(topic, Vector3Stamped, queue_size=10)

    def msg_process(self, msg):
        if(msg.topic == "tags"):
            msg_decode=json.loads(msg.payload)
            msg_dict = msg_decode[0]

            if(int(msg_dict["tagId"]) == self.pozyx_tag):
                is_success = msg_dict["success"]
                if(is_success):
                    pos_message = Vector3Stamped()
                    pos_message.vector.x = float(msg_dict["data"]["coordinates"]["x"])
                    pos_message.vector.y = float(msg_dict["data"]["coordinates"]["y"])
                    pos_message.vector.z = float(msg_dict["data"]["coordinates"]["z"])

                    pozyx_stamp = float(msg_dict["timestamp"])
                    #print(int(pozyx_stamp))
                    #print(pozyx_stamp - int(pozyx_stamp))
                    pos_message.header.stamp.secs = int(pozyx_stamp)
                    pos_message.header.stamp.nsecs = int((pozyx_stamp - int(pozyx_stamp))*math.pow(10,9))

                    self.pos_publisher.publish(pos_message)
#        else:
#            print  msg.topic + " is not a supported topic"


def main():
    rospy.init_node('pozyx_bridge', anonymous=True)
    pozyx_sub = pozyx_bridge(0x6063, '#', 'tags', host='172.25.224.1')

    rospy.on_shutdown(pozyx_sub.hook)

    while not rospy.is_shutdown():
        pozyx_sub.looping()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
