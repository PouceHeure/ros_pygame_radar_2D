#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty

from ros_pygame_radar_2D.pygame_radar_2D.pygame_radar_2D import radar, point


class RadarRos: 

    def __init__(self,topic):
        self.i = 0
        rospy.Subscriber(topic, Empty, self.callback, queue_size=10)
        self.radar = radar.Radar(300,r_lim=300)
        self.radar.start()
    
    def callback(self,data):
        self.radar.set_points([point.PointPolar(self.i,self.i)])
        self.i += 10
    
if __name__ == '__main__': 
    rospy.init_node('listener', anonymous=True)
    RadarRos("test")
    rospy.spin()