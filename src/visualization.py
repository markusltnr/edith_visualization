#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import PointCloud2
import time
import sched
import open3d as o3d
import numpy as np
from edith_msgs.msg import Object, CandidateObject, ObjectMatch, ObjectStateClass
from mongodb_store.message_store import MessageStoreProxy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from open3d_ros_helper import open3d_ros_helper as orh

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
        self.marker_publisher = rospy.Publisher(self.name+'/marker', MarkerArray, queue_size = 10)
        self.candidate_publisher = rospy.Publisher(self.name+'/candidate_objects', PointCloud2, queue_size=10)
        self.permanent_publisher = rospy.Publisher(self.name+'/permanent_objects', PointCloud2, queue_size=10)

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
        rgb = np.asarray((r/255.0, g/255.0, b/255.0), dtype=np.float64)
        
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
        self.pcl = o3d.geometry.PointCloud()
        if self.running == 1: 
            markers = []

            permanent_objects = None
            for obj in object_list:
                cloud = orh.rospc_to_o3dpc(obj.obj_cloud_no_normals)

                self.color_pcl(cloud, (255, 0, 0))
                if permanent_objects is None:
                    permanent_objects = np.asarray(cloud.points)
                    permanent_objects_colors = np.asarray(cloud.colors)
                else:
                    permanent_objects = np.concatenate((permanent_objects, np.asarray(cloud.points)), axis=0)
                    permanent_objects_colors = np.concatenate((permanent_objects_colors, np.asarray(cloud.colors)), axis=0)
                

            candidate_objects = None
            for candidateObject in candidateObject_list:
                #rospy.loginfo(candidateObject.state)
                #self.pcl_segment = convertCloudFromRosToOpen3d(candidateObject.object.obj_cloud)
                self.pcl_segment = orh.rospc_to_o3dpc(candidateObject.object.obj_cloud_no_normals, remove_nans=True)
                if candidate_objects is None:
                    candidate_objects = np.asarray(self.pcl_segment.points)
                    candidate_objects_colors = np.asarray(self.pcl_segment.colors)
                else:
                    candidate_objects = np.concatenate((candidate_objects, np.asarray(self.pcl_segment.points)), axis=0)
                    candidate_objects_colors = np.concatenate((candidate_objects_colors, np.asarray(self.pcl_segment.colors)), axis=0)
                if candidateObject.state == ObjectStateClass.NEW:
                    self.color_pcl(self.pcl_segment, (0, 200, 0))
                # elif candidateObject.state == ObjectStateClass.REMOVED:
                #     self.color_pcl(self.pcl_segment, (255, 0, 0))
                elif candidateObject.state == ObjectStateClass.DISPLACED:
                    color_flag = False
                    center = self.pcl_segment.get_center()
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.id = candidateObject.match.model_id
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.type = 0
                    start = Point()
                    for obj in object_list:
                        if obj.id == candidateObject.match.model_id:
                            model = orh.rospc_to_o3dpc(obj.obj_cloud_no_normals)
                            center_model = model.get_center()
                            start.x = center_model[0]
                            start.y = center_model[1]
                            start.z = center_model[2]
                            self.color_pcl(orh.rospc_to_o3dpc(obj.obj_cloud_no_normals), (255, 0, 255))
                    #start.x = center[0]-candidateObject.match.transform.translation.x
                    #start.y = center[1]-candidateObject.match.transform.translation.y
                    #start.z = center[2]-candidateObject.match.transform.translation.z
                    end = Point()
                    end.x = center[0]
                    end.y = center[1]
                    end.z = center[2]
                    marker.scale.x = 0.01
                    marker.scale.y = 0.05
                    marker.scale.z = 0.1
                    

                    marker.points.append(start)
                    marker.points.append(end)
                    markers.append(marker)
                    self.color_pcl(self.pcl_segment, (255, 200, 0))
                    # for color in self.color_list:
                    #     c, rgb = color
                    #     if c == candidateObject.object.id:
                    #         self.color_pcl(self.pcl_segment, rgb)
                    #         color_flag = True

                    # if color_flag is False:
                    #     r = np.random.randint(0,200)
                    #     g = np.random.randint(0,200)
                    #     b = np.random.randint(100,255)
                    #     self.color_pcl(self.pcl_segment, (r, g, b))
                    #     print((r,g,b))
                    #     self.color_list.append((candidateObject.object.id, (r, g, b)))
                elif candidateObject.state == ObjectStateClass.STATIC:
                    self.color_pcl(self.pcl_segment, (255, 255, 255))
                elif candidateObject.state == ObjectStateClass.UNKNOWN:
                    self.color_pcl(self.pcl_segment, (0, 0, 255))
                
                self.marker_publisher.publish(markers)

                    

                
        if self.running >= 1: 
            # don't publish empty point cloud because rviz will change the color transformer from rgb to intensity
            if self.running > 1:
                self.pcl.points.append(np.asarray((0, 0, 0), dtype=np.float64))
                self.pcl.colors.append(np.asarray((0, 0, 0), dtype=np.float64))
            self.p_pcl = orh.o3dpc_to_rospc(self.pcl, "map")
            pcd1 = o3d.geometry.PointCloud()
            pcd1.points = o3d.utility.Vector3dVector(permanent_objects)
            pcd1.colors = o3d.utility.Vector3dVector(permanent_objects_colors)  
            perm = orh.o3dpc_to_rospc(pcd1, "map")
            pcd2 = o3d.geometry.PointCloud()
            if candidate_objects is not None:
                pcd2.points = o3d.utility.Vector3dVector(candidate_objects)
                pcd2.colors = o3d.utility.Vector3dVector(candidate_objects_colors)   
            cand = orh.o3dpc_to_rospc(pcd2, "map")
            self.seq += 1
            self.visualize_publisher.publish(self.p_pcl)
            self.permanent_publisher.publish(perm)
            self.candidate_publisher.publish(cand)
 
if __name__ == '__main__':
    rospy.init_node('edith_visualization_service')
    service = VisualizationService(rospy.get_name())
    rospy.spin()
