#!/usr/bin/env python

# Import modules
from pcl_helper import *
import tf

def pcl_ros():
    #pcl_pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
    rospy.init_node('clustering', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #cloud = pcl.load_XYZRGB('tabletop.pcd')
       # ros_data = pcl_to_ros(cloud)
        #pcl_pub.publish(ros_data)
        br = tf.TransformBroadcaster()
        br.sendTransform((0.175, -0.64, 0.495),
                         tf.transformations.quaternion_from_euler(0, 3.14/9, 3.14/2),
                         rospy.Time.now(),
                         "zed_left_camera",
                         "base_link")
        bc = tf.TransformBroadcaster()
        bc.sendTransform((0.175, -0.64, 0.495),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "world",
                         "zed_left_camera")
        rate.sleep()


if __name__ == '__main__':

    try:
        pcl_ros()
    except rospy.ROSInterruptException:
        pass
