#!/usr/bin/env python3
import rospy

from ros_pygame_radar_2D.msg import PointPolar, RadarPointCloud
from ros_pygame_radar_2D.pygame_radar_2D.pygame_radar_2D import radar, point

class RadarRos: 

    def __init__(self,topic):
        self.i = 0
        rospy.Subscriber(topic, RadarPointCloud, self.callback, queue_size=10)
        self.radar = radar.Radar(300,r_lim=300)
        self.radar.start()
    
    def callback(self,data):
        points = []
        for point in data.points: 
            new_point = PointPolar(point.theta,point.r)
            points.append(new_point)
        self.radar.set_points(points)
        self.i += 10
    
if __name__ == '__main__': 
    rospy.init_node('listener', anonymous=True)
    RadarRos("test")
    rospy.spin()