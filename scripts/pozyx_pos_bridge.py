#!/usr/bin/python

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
import mqtt_ros_bridge.bridge as bridge
import rospy
import signal
import json
import math

class pozyx_bridge(bridge.bridge):

    def __init__(self, tags, mqtt_topic, client_id = "",user_id = "",password = "", host = "localhost", port = 1883, keepalive = 60):
        self.tags = tags
        self.pos_publishers = {}
        for tag in tags:
            if isinstance(tag, int):
                ros_topic = '~' +hex(tag)
                self.pos_publishers[tag] = rospy.Publisher(ros_topic, PointStamped, queue_size=10)
            else:
                print(tag + " is not a valid tag number. Check configuration.")
        bridge.bridge.__init__(self,mqtt_topic, client_id,user_id,password, host, port, keepalive)


    def msg_process(self, msg):
        if(msg.topic == "tags"):
            msg_decode=json.loads(msg.payload)
            msg_dict = msg_decode[0]

            if(int(msg_dict["tagId"]) in self.tags):
                is_success = msg_dict["success"]
                if(is_success):
                    pos_message = PointStamped()
                    pos_message.point.x = float(msg_dict["data"]["coordinates"]["x"])/1000
                    pos_message.point.y = float(msg_dict["data"]["coordinates"]["y"])/1000
                    pos_message.point.z = float(msg_dict["data"]["coordinates"]["z"])/1000

                    pozyx_stamp = float(msg_dict["timestamp"])
                    #print(int(pozyx_stamp))
                    #print(pozyx_stamp - int(pozyx_stamp))
                    pos_message.header.stamp.secs = int(pozyx_stamp)
                    pos_message.header.stamp.nsecs = int((pozyx_stamp - int(pozyx_stamp))*math.pow(10,9))

                    self.pos_publishers[int(msg_dict["tagId"])].publish(pos_message)
#        else:
#            print  msg.topic + " is not a supported topic"


def main():
    rospy.init_node('pozyx_bridge', anonymous=True)

    pozyx_server_ip = rospy.get_param('~pozyx_server_ip', 'localhost')
    pozyx_tags = rospy.get_param('~pozyx_tags', [])


    print("Server IP: " + pozyx_server_ip)
    print("Pozyx tags to subscribe:")
    for tag in pozyx_tags:
        if isinstance(tag,int):
            print("\t" + hex(tag))

    pozyx_sub = pozyx_bridge(pozyx_tags, '#', host=pozyx_server_ip)

    rospy.on_shutdown(pozyx_sub.hook)

    # connect
    pozyx_sub.connect_async()
    pozyx_sub.start_looping()

    rospy.spin()

    # stop looping
    pozyx_sub.stop_looping()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
