#!/usr/bin/env python

# Import modules
from pcl_helper import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
import time
import math
import tf2_ros
import tf2_geometry_msgs

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):




    # TODO: Convert ROS msg to PCL data
   #coud = ros_to_pcl(pcl_msg)
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.0026
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    cloud_filtered = vox.filter()
    #filename = 'voxel_downsampled.pcd'
    #pcl.save(cloud_filtered, filename)

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()

    filter_axis= 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.0
    axis_max = 0.7
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()

    filter_axis= 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.0
    axis_max = 0.4
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()


    filter_axis= 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 1.15
    axis_max = 1.5
    passthrough.set_filter_limits(axis_min, axis_max)

    cloud_filtered = passthrough.filter()


    #filename = 'pass_through_filtered.pcd'
    #pcl.save(cloud_filtered, filename)


    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.0143
    seg.set_distance_threshold(max_distance)

    inliers, coefficients = seg.segment()


    # TODO: Extract inliers and outliers

    extracted_inliers = cloud_filtered.extract(inliers, negative=True)
    extracted_outliers = cloud_filtered.extract(inliers, negative=False)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(extracted_inliers)                     # Apply function to convert XYZRGB to XYZ (to reduce the computational burden)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.06)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(5000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately    
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []
    centroid = []
    white_cloud_array = white_cloud.to_array()

    for j, indices in enumerate(cluster_indices):
        centroids = []
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
            centroids.append([white_cloud_array[indice][0],
                                            white_cloud_array[indice][1],
                                            white_cloud_array[indice][2]])
 
        aux_centroid = np.mean(centroids, axis=0)[:3]
        centroid.append(aux_centroid)

        pick_pose = PoseStamped()
        pick_poses = PoseArray()
        pick_pose.header.frame_id = "world"
         

    for k, positions in enumerate(centroid):

        pick_pose.pose.position.x = float(positions[0]) 
        pick_pose.pose.position.y = float(positions[1])
        pick_pose.pose.position.z = float(positions[2])
        pick_pose.pose.orientation.x = float(0.0)
        pick_pose.pose.orientation.y = float(0.0)
        pick_pose.pose.orientation.z = float(0.0)
        pick_pose.pose.orientation.w = float(0.0)
        transform = tf_buffer.lookup_transform("base_link",
                                       pick_pose.header.frame_id, #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

        transd_pose = tf2_geometry_msgs.do_transform_pose(pick_pose, transform)
        pcl_objects_pose.publish(transd_pose)
        time.sleep(3)

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(extracted_inliers)
    ros_cloud_table = pcl_to_ros(extracted_outliers)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    #pcl_sub = rospy.Subscriber("/zed/point_cloud/cloud_registered", pc2.PointCloud2, pcl_callback, queue_size=1)
    pcl_sub = rospy.Subscriber("/camera/depth_registered/points", pc2.PointCloud2, pcl_callback, queue_size=1)


    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    pcl_objects_pose= rospy.Publisher("/pcl_pose", PoseStamped, queue_size=1)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
