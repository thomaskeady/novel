import rospy
from novel_msgs.msg import NovelObject, NovelObjectArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point
import numpy as np
import math
from copy import deepcopy
class NovelLidarDetection(object):
    def __init__(self, 
        in_scan_topic='/scan', 
        expected_scan_topic='/expected_scan', 
        out_scan_topic='filtered_scan', 
        detected_object_topic='/lidar_objects', 
        amcl_pose_topic='/amcl_pose',
        threshold=0.05, 
        window_size=3, 
        window_step=2):
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
        self.out_scan_pub = rospy.Publisher(out_scan_topic, LaserScan, queue_size=5)
        self.detected_object_pub = rospy.Publisher(detected_object_topic, NovelObjectArray, queue_size=5)
        self.last_scan =  np.array([])
        self.last_expected =  np.array([])
        self.last_pose = Pose()
        self.last_scan_msg = LaserScan()
        self.window_size = window_size
        self.window_step = window_step
        self.threshold = threshold
        self.range_max = 1
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
        num_windows = math.floor(float(len(a) - wsz + 1)/wsp)
        indexer = np.arange(wsz)[None, :] + wsp*np.arange(num_windows)[:, None]
        indexer = indexer.astype(np.int)
        return (a[indexer], indexer)
    def detect(self):
        """
        Detect objcets based on last scans

        """
        
        if np.array_equal(self.last_expected,self.last_scan):
            return
        if not any(self.last_expected):
            self.out_scan_pub.publish(self.last_scan_msg)
            return
        
        if self.last_scan.shape != self.last_expected.shape:
            rospy.logerr('Expected scan is not the same size as received scan')
        sw, sw_i = self.window_stack(self.last_scan)
        ew, ew_i = self.window_stack(self.last_expected)
        detected_objects = np.zeros(self.last_scan.shape, dtype=np.bool)
        for s,e,i in zip(sw, ew, sw_i):
            # er = np.correlate(s,e)
            er = ((s-e)**2).mean()
            if er > self.threshold:
                detected_objects[i] = True
        # Calculate center of mass of objects
        detected_objects_position = []
        in_object = False
        pos_begin, pos_end, i = 0, 0, 0
        while i < len(detected_objects):
            if detected_objects[i]:
                if not in_object:
                    pos_begin = i
                    in_object = True
            else:
                if in_object:
                    pos_end = i
                    com = math.floor((pos_begin+pos_end)/2)
                    detected_objects_position.append(int(com))
                    in_object = False
            i += 1
        msg = NovelObjectArray()
        for i,o in enumerate(detected_objects_position):
            
            m = NovelObject()
            p = self.calculate_position_from_index(o)
            m.pose.pose = p
            # Make this covariance better?
            m.pose.covariance = list(np.zeros((6,6)).flatten())
            m.id = i
            msg.detected_objects.append(m)
        self.detected_object_pub.publish(msg)
        # Publish filtered laser scan
        msg = deepcopy(self.last_scan_msg)
        u_range = np.array(msg.ranges)
        real_expected = self.last_expected*self.range_max
        u_range[detected_objects] = real_expected[detected_objects]
        msg.ranges = list(u_range)
        msg.intensities = []
        self.out_scan_pub.publish(msg)

        

    def calculate_position_from_index(self, median_index):
        z = self.last_pose.position.z
        last_scan = self.last_scan
        rad = float(median_index)/len(last_scan) * 2 * math.pi
        x = last_scan[median_index] * math.cos(rad) * self.range_max
        y = last_scan[median_index] * math.sin(rad) * self.range_max
        return Pose(position=Point(x,y,z))
    def pose_callback(self, msg):
        self.last_pose = msg.pose.pose
        self.last_pose_covariance = msg.pose.covariance
        self.last_pose_header = msg.header
    def ls_callback(self, msg):
        self.range_max = msg.range_max
        self.last_scan = np.array(msg.ranges)/self.range_max
        self.last_scan[self.last_scan == np.inf] = 0
        self.last_scan_msg = msg
    def els_callback(self, msg):
        self.last_expected = np.array(msg.ranges)/self.range_max
        self.last_expected[self.last_expected == np.inf] = 0