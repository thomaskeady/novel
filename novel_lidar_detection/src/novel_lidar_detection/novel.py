import rospy
from novel_msgs.msg import NovelObject, NovelObjectArray
from sensor_msgs.msg import LaserScan
from geometry_msgs import PoseWithCovarianceStamped, Pose, Point
import numpy as np
import math
class NovelLidarDetection(object):
    def __init__(self, 
        in_scan_topic='/scan', 
        expected_scan_topic='/expected_scan', 
        out_scan_topic='filtered_scan', 
        detected_object_topic='/lidar_objects', 
        pose_topic='/amcl_pose',
        threshold=1, 
        window_size=3, 
        window_step=1):
        """
        Initializes object 

        params
        ------
        in_scan_topic : str
            Real scan topic
        expected_scan_topic : str
            Expected scan topic
        out_scan_topic : str
            Filtered scan topic to go to localization nodes
        detected_object_topic : str
            Topic to publish detected objects to
        pose_topic : str
            Topic to subscribe to pose
        threshold: float
            Error threshold from cross correlation to count as a new object
        window_size: int
            Sliding window size. Window is slid across both signals, getting an cross correlation error
        window_step: int
            Window step size.
        """
        rospy.Subscriber(in_scan_topic, LaserScan, self.ls_callback)
        rospy.Subscriber(expected_scan_topic, LaserScan, self.els_callback)
        rospy.Subscriber(amcl_pose_topic, PoseWithCovarianceStamped, self.pose_callback)
        self.out_scan_pub = rospy.Publisher(out_scan_topic, LaserScan)
        self.detected_object_pub = rospy.Publisher(detected_object_topic, NovelObjectArray)

        self.window_size = window_size
        self.window_step = window_step
        self.threshold = threshold
    def window_stack(self, a):
        '''
        Function from here:
            https://stackoverflow.com/questions/15722324/sliding-window-in-numpy
        Returns a sliding window over a sample, a
        params
        ------
        a : np.array(m,n)
            Sample to slide over

        returns
        -------
        np.array(m,d)
            A stack of windowed samples, where d is the window size
        np.array(m,d)
            An array of the same size that gives the indicies of the windowed data
        '''
        wsz = self.window_size
        wsp = self.window_step
        num_windows = math.ceil(len(a)/ws)
        indexer = np.arange(ws)[None, :] + wsp*np.arange(num_windows)[:, None]
        return (a[indexer], indexer)
    def detect(self):
        """
        Detect objcets based on last scans

        """
        
        if self.last_scan.shape != self.last_expected.shape:
            rospy.logerr('Expected scan is not the same size as received scan')
        sw, sw_i = self.window_stack(self.last_scan)
        ew, ew_i = self.window_stack(self.last_expected)
        detected_objects = np.zeros(self.last_scan.shape, dtype=np.bool)
        for s,e,i in zip(sw, ew, swi):
            er = np.correlate(s,e)
            if er > self.threshold:
                detected_objects[i] = True
        # Calculate center of mass of objects
        detected_objects_position = []
        in_object = False
        pos_begin, pos_end, i = 0
        while i < len(detected_objects):
            if detected_objects[i]:
                if not in_object:
                    pos_begin = i
                    in_object = True
            else:
                if in_object:
                    pos_end = i
                    com = math.floor((pos_begin+pos_end)/2)
                    detected_objects_position.append(com)
                    in_object = False
        msg = NovelObjectArray
        for o,i in enumerate(detected_objects_position):
            m = NovelObject
            p = self.calculate_position_from_index(o)
            m.pose.pose = p
            # Make this covariance better?
            m.pose.covariance = np.zeros(4).flatten() 
            m.id = i
            msg.objects.append(m)
        self.detected_object_pub.publish(msg)
        
        # TODO estimate 

    def calculate_position_from_index(self, median_index, last_scan):
        z = self.last_pose.z
        rad = median_index/len(last_scan) * 2 * math.pi
        x = last_scan[median_index] * math.cos(rad)
        y = last_scan[median_index] * math.sin(rad)
        return Pose(Point(x,y,z))
    def pose_callback(self, msg):
        self.last_pose = msg.pose.pose
        self.last_pose_covariance = msg.pose.covariance
        self.last_pose_header = msg.header
    def ls_callback(self, msg):
        self.last_scan = np.array(msg.ranges)
    def els_callback(self, msg):
        self.last_expected = np.array(msg.ranges)
    