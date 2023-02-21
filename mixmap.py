#!/usr/bin/env python
'''
The Python program of collision check for jackal
input       : /map (nav_msgs/OccupancyGrid)  
            : /scan(nav_msgs/Odometry)
            : /tf

output topic: /mixmap (nav_msgs/OccupancyGrid)

Author: Shusen Lin
'''
import rospy
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

class MixMapROS:
    def __init__(self):
        rospy.init_node('MixMapping', anonymous=False)
        self.map_last_publish = rospy.Time()
        self.map_frame        = rospy.get_param('~map_frame', '/map')
        self.lidar_frame       = rospy.get_param('~lidar_frame', '/os_sensor')    
        self.map_in_topic     = rospy.get_param('~origin_map', '/map')
        self.scan_in_topic    = rospy.get_param('~scan_in', '/scan')
        self.angle_range      = rospy.get_param('~check_angle_range', 30) #in degree
        self.linear_range     = rospy.get_param('~check_linear_range', 2.0)#in meter

        # Creata a OccupancyGrid message template
        self.map_resolution  = None
        self.map_out = OccupancyGrid()
        self.map_out.header.frame_id = None
        self.map_out.info.resolution = None
        self.map_out.info.width  = None
        self.map_out.info.height = None
        self.map_out.info.origin.position.x = None
        self.map_out.info.origin.position.y = None

        self.map_in = None
        self.scan_in = None
        self.map_in_sub  = rospy.Subscriber(self.map_in_topic, OccupancyGrid, self.mapCallback,queue_size=2)
        self.scan_in_sub = rospy.Subscriber(self.scan_in_topic,LaserScan,     self.scanCallback,queue_size=2)

        self.mixmap_pub  = rospy.Publisher('mixmap' , OccupancyGrid, queue_size=2)


    def process_points(self, point):
        '''
    	Process one point and update the gridmap using the point
    	
    	:param point: one point 	
    	'''
        assert True

        idx = round((point[0] - self.map_out.info.origin.position.x) / self.map_resolution)
        jdx = round((point[1] - self.map_out.info.origin.position.y) / self.map_resolution)
        ijdx = idx + jdx*self.map_out.info.height
        self.map_out.data[ijdx] = 100

    def mapCallback(self, data):       
        try:
            self.map_in = data
            self.map_resolution = data.info.resolution
            self.map_out.header.frame_id = data.header.frame_id
            self.map_out.info.resolution = data.info.resolution
            self.map_out.info.width  = data.info.width
            self.map_out.info.height = data.info.width
            self.map_out.info.origin.position.x = data.info.origin.position.x
            self.map_out.info.origin.position.y = data.info.origin.position.y
            # self.map_out.data = np.copy(list(data.data))
            self.map_out.data = list(data.data)

        except ValueError:
            print('Receive empty map, still waiting...')
            pass

    def scanCallback(self,data):
        try:
            self.scan_in = data
            if (self.map_in is not None) and (self.scan_in is not None):
                self.mixmap()
                self.publish_occupancygrid(data.header.stamp)        

        except ValueError:
            print("Receive empty scan, still waiting...")
            pass

    def transform_matrix_2d(self,theta):
        return np.array([[np.cos(theta), -1 * np.sin(theta)],
                         [np.sin(theta),      np.cos(theta)]],)

    def mixmap(self):
        '''
        The main function to mix the origin map with fast_lidar map
        '''
        listener = tf.TransformListener()
        num_scan = len(self.scan.ranges)
        center_index = np.round(num_scan/2)
        min_index = int(np.round(center_index - num_scan*self.angle_range/len(self.scan_in.ranges)))
        max_index = int(np.round(center_index + num_scan*self.angle_range/len(self.scan_in.ranges)))
        added_angle = self.scan.angle_increment

        try:
            listener = tf.TransformListener()
            listener.waitForTransform("os_lidar", "map", rospy.Time(), rospy.Duration(0.5))
            lidar_position, lidar_quaternion = listener.lookupTransform( self.map_frame , self.lidar_frame ,rospy.Time(0))

            robot_pose_x = lidar_position[0]
            robot_pose_y = lidar_position[1]
            robot_pose_t = euler_from_quaternion(lidar_quaternion)[2]
            
            print(robot_pose_x,robot_pose_y,robot_pose_t)

            in_range_point = []
            for idx in range(min_index,max_index+1):
                if self.scan_in.ranges[idx]<=self.linear_range:
                    new_angle = (idx*added_angle - np.pi) + robot_pose_t 
                    px=float(self.scan.ranges[idx])*np.cos(new_angle)
                    py=float(self.scan.ranges[idx])*np.sin(new_angle)
                    p_mx = px + robot_pose_x
                    p_my = py + robot_pose_y
                    in_range_point.append([p_mx,p_my])

            DEBUG = np.array(in_range_point)
            if DEBUG.size != 0:
                print('update the mixmap')
                np.apply_along_axis(self.process_points, axis = 1, arr = in_range_point)

        except (tf.LookupException, tf.ConnectivityException):
            pass

    def publish_occupancygrid(self, stamp):
        self.map_out.header.stamp = stamp
        self.mixmap_pub.publish(self.map_out)
        rospy.loginfo_once("Publish Mixmap")

if __name__ == "__main__": 
    voxblox_map = MixMapROS()
    while not rospy.is_shutdown():
    	rospy.spin()
