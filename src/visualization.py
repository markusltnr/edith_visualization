#!/usr/bin/env python
 
import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import PointCloud2
import time
import sched
import open3d
import numpy as np
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d #https://github.com/felixchenfy/open3d_ros_pointcloud_conversion
from obj_det_ppf_matching_msgs.msg import Object, CandidateObject, ObjectMatch, ObjectStateClass
from mongodb_store.message_store import MessageStoreProxy

class VisualizationService(object):
    def __init__(self, name):
        self.name = name
        rospy.loginfo('%s:  Start visalization service', self.name)

        self.msg_store = MessageStoreProxy()

        rospy.Service(self.name+'/visualize',Empty,self.visualize)
        rospy.Service(self.name+'/clear',Empty,self.clear)
        rospy.Service(self.name+'/stop',Empty,self.stop)

        self.visualize_publisher = rospy.Publisher(
            self.name+'/visualization', PointCloud2, queue_size=1)

        self.running = 0
        self.seq = 1

        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.setup(5, self.create_full_pcl)
        self.color_list = []
        self.scheduler.run()
        
    def setup(self, interval, action, actionargs=()):
        action(*actionargs)
        self.event = self.scheduler.enter(interval, 1, self.setup, (interval, action, actionargs))

    def visualize(self, goal):
        self.running = 1
        rospy.loginfo('%s: Publish Pointcloud', self.name)
        return EmptyResponse()

    def clear(self, goal):
        self.running = 2
        rospy.loginfo('%s: Publish Empty Pointcloud', self.name)
        return EmptyResponse()

    def stop(self, goal):
        self.running = 0
        rospy.loginfo('%s: Stop Publish Pointcloud', self.name)
        return EmptyResponse()

    def color_pcl(self, pcl_segment, rgb):
        r, g, b = rgb
        rgb = np.asarray((r/255, g/255, b/255), dtype=np.float64)
        
        for point in pcl_segment.points:
            p = np.asarray((point[0], point[1], point[2]), dtype=np.float64)
            self.pcl.points.append(p)
            self.pcl.colors.append(rgb)

    def create_full_pcl(self):
        if rospy.is_shutdown():
            if self.scheduler and self.event:
                self.scheduler.cancel(self.event)

        candidateObject_list = []
        object_list = []

        for msg, meta in self.msg_store.query(CandidateObject._type):
            candidateObject_list.append(msg)

        for msg, meta in self.msg_store.query(Object._type):
            object_list.append(msg)  

        self.pcl = None
        self.pcl = open3d.geometry.PointCloud()
        if self.running == 1: 

            for obj in object_list:
               self.color_pcl(convertCloudFromRosToOpen3d(obj.obj_cloud), (255, 0, 255))

            for candidateObject in candidateObject_list:
                #rospy.loginfo(candidateObject.state)
                self.pcl_segment = convertCloudFromRosToOpen3d(candidateObject.object.obj_cloud)
                if candidateObject.state == ObjectStateClass.NEW:
                    self.color_pcl(self.pcl_segment, (0, 255, 0))
                elif candidateObject.state == ObjectStateClass.REMOVED:
                    self.color_pcl(self.pcl_segment, (255, 0, 0))
                elif candidateObject.state == ObjectStateClass.DISPLACED:
                    color_flag = False
                    for color in color_list:
                        c, rgb = color
                        if c == candidateObject.object.id:
                            self.color_pcl(self.pcl_segment, rgb)
                            color_flag = True

                    if color_flag is False:
                        r = np.random.randint(0,200)
                        g = np.random.randint(0,200)
                        b = np.random.randint(100,255)
                        self.color_pcl(self.pcl_segment, (r, g, b))
                        self.color_list.append((candidateObject.object.id, (r, g, b)))
                elif candidateObject.state == ObjectStateClass.STATIC:
                    self.color_pcl(self.pcl_segment, (255, 255, 255))
                elif candidateObject.state == ObjectStateClass.UNKNOWN:
                    self.color_pcl(self.pcl_segment, (0, 0, 255))
                
        if self.running >= 1: 
            # don't publish empty point cloud because rviz will change the color transformer from rgb to intensity
            if self.running > 1:
                self.pcl.points.append(np.asarray((0, 0, 0), dtype=np.float64))
                self.pcl.colors.append(np.asarray((0, 0, 0), dtype=np.float64))
            self.p_pcl = convertCloudFromOpen3dToRos(self.pcl, self.seq, "map")
            self.seq += 1
            self.visualize_publisher.publish(self.p_pcl)
 
if __name__ == '__main__':
    rospy.init_node('edith_visualization_service')
    service = VisualizationService(rospy.get_name())
    rospy.spin()