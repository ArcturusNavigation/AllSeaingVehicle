#!/usr/bin/env python

import numpy as np 
import rospy 
from sensor_suite.msg import Object, ObjectArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

class ObjectTracker:
    def __init__(self,obj):
        self.label = obj.label
        self.pos = obj.pos
        self.pos_count = 1
    
    def matches(self, obj):
        return self.label == obj.label and self.distance(obj) < .3 # in meters

    def update(self, pos):
        self.pos.x = (self.pos.x * self.pos_count + pos.x) / (self.pos_count + 1)
        self.pos.y = (self.pos.y * self.pos_count + pos.y) / (self.pos_count + 1)
        self.pos_count += 1

    def distance(self, obj):
        return np.sqrt((self.pos.x - obj.pos.x)**2 + (self.pos.y - obj.pos.y)**2)

    def __str__(self):
        return "TrackedObject(" + str(self.label) + "," + str(self.pos) + ")"

    def __repr__(self):
        return self.__str__()

class MappingNode:
    def __init__(self):
        self.tracked_objects = []
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'world'
        self.grid.header.stamp = rospy.Time.now()
        self.grid.info.resolution = .5
        self.grid.info.width = 600 
        self.grid.info.height = 600
        self.grid.info.origin.position.x = -300
        self.grid.info.origin.position.y = -300
        self.grid.info.origin.position.z = 0
        self.objects_sub = rospy.Subscriber('/sensor_suite/objects', ObjectArray, self.objects_callback)
        self.map_pub = rospy.Publisher('/map', ObjectArray, queue_size=10) 
        self.grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)
        self.marker_pub = rospy.Publisher('/map_markers', MarkerArray, queue_size=10)

    def objects_callback(self, objs):
        i,j = 0,0 
        while i < len(objs.objects):
            found_match = False 
            while j < len(self.tracked_objects):
                if self.tracked_objects[j].matches(objs.objects[i]):
                    self.tracked_objects[j].update(objs.objects[i].pos)
                    i += 1
                    j += 1
                    found_match = True
                    break
                j += 1
            if j == len(self.tracked_objects) and not found_match:
                self.tracked_objects.append(ObjectTracker(objs.objects[i]))
                i += 1
        print(self.tracked_objects)
        map = ObjectArray()
        map.objects = self.tracked_objects
        self.map_pub.publish(map)
        self.publish_markers()
        self.publish_grid()
    
    def publish_grid(self):
        self.grid.data = np.zeros((600,600))
        for obj in self.tracked_objects:
            x = int(obj.pos.x / self.grid.info.resolution)
            y = int(obj.pos.y / self.grid.info.resolution)
            self.grid.data[x,y] = 100 #TODO: Cover full area of buoy + some
        self.grid.data = list(self.grid.data.flatten())
        self.grid_pub.publish(self.grid)

    def publish_markers(self):
        markers = MarkerArray()
        count = 0
        for obj in self.tracked_objects:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'objects'
            marker.id = count 
            count+=1 
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = obj.pos.x
            marker.pose.position.y = obj.pos.y
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = .2
            marker.scale.y = .2
            marker.scale.z = .2
            marker.color.r = 1
            marker.color.g = 0
            marker.color.b = 0
            marker.color.a = 1
            marker.lifetime = rospy.Duration(0)
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

if __name__ == '__main__':
    rospy.init_node('mapping_node')
    node = MappingNode()
    rospy.spin()