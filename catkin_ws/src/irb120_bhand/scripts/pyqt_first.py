from __future__ import division
import sys
from PyQt4 import QtCore, QtGui, uic
from pcl_helper import *
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
import time
import math
import tf2_ros
import tf2_geometry_msgs

qtCreatorFile = "tax_calc.ui" # Enter file here.

Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)

def pcl_callback(pcl_msg):

    app = QtGui.QApplication(sys.argv)
    window = MyApp()
    window.show()
    sys.exit(app.exec_())

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





class MyApp(QtGui.QMainWindow, Ui_MainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.calc_tax_button.clicked.connect(self.pcl_ros)


    def pcl_ros(teste1,teste2):
    #pcl_pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
        rospy.init_node('clustering', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
        #cloud = pcl.load_XYZRGB('tabletop.pcd')
       # ros_data = pcl_to_ros(cloud)
        #pcl_pub.publish(ros_data)
            br = tf.TransformBroadcaster()
            br.sendTransform((0.04, -0.78, 0.97),
                         tf.transformations.quaternion_from_euler(0, 3.14/4.2, 3.14/2),
                         rospy.Time.now(),
                         "camera_link",
                         "base_link")
            bc = tf.TransformBroadcaster()
            bc.sendTransform((0.08, -0.78, 0.97),
                         tf.transformations.quaternion_from_euler((-3.14/2 - 3.14/4.2), 0, 0),
                         rospy.Time.now(),
                         "world",
                         "base_link")
            rate.sleep()
    
    #def CalculateTax(self):
        #price = int(self.price_box.toPlainText())
        #tax = (self.tax_rate.value())
        #total_price = price  + ((tax / 100) * price)
        #total_price_string = "The total price with tax is: " + str(total_price)
        #self.results_window.setText(total_price_string)
        
if __name__ == "__main__":
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

  