#!/usr/bin/env python3
import rospy

# import msgs
from ros_pygame_radar_2D.msg import PointPolar, RadarPointCloud

# import radar 
from ros_pygame_radar_2D.pygame_radar_2D.pygame_radar_2D import radar, point

class RadarRos: 

    def __init__(self,topic,size,**kwargs):
        rospy.Subscriber(topic, RadarPointCloud, self.callback, queue_size=10)
        self.radar = radar.Radar(size,**kwargs)
        
    def start(self):
        self.radar.start()

    def stop(self):
        self.radar.quit()
    
    def callback(self,data):
        points = []
        for p in data.points: 
            new_point = point.PointPolar(p.theta,p.r,theta_in_deg=False)
            points.append(new_point)
        self.radar.set_points(points)
    
if __name__ == '__main__': 
    rospy.init_node('radar_node', anonymous=True)
    radar_ros = RadarRos("radar",1000,r_lim=1,point_size=10)
    radar_ros.start()
    rospy.spin()
    radar_ros.stop()