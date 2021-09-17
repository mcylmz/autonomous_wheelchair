#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/grid_minimum.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <math.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <dynamic_reconfigure/server.h>
#include <perception/ObjectSegmenterConfig.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/shadowpoints.h>

#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/Marker.h>

geometry_msgs::TransformStamped static_transform;

ros::Publisher pass_pub;
ros::Publisher grid_pub;
ros::Publisher proj_pub;
ros::Publisher noshadow_pub;
ros::Publisher vis_pub;
ros::Publisher final_pub;

void setMarker(const Eigen::Vector3f &position, int32_t id, int32_t action=visualization_msgs::Marker::ADD)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CYLINDER;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "clusters";
    marker.id = id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    vis_pub.publish(marker);
}

segmenter_reconfig::ObjectSegmenterConfig reconf;
void reconf_cb(segmenter_reconfig::ObjectSegmenterConfig &config, uint32_t level) {
    reconf = config;
}

void doTransform(const sensor_msgs::PointCloud2 &p_in, sensor_msgs::PointCloud2 &p_out, const geometry_msgs::TransformStamped& t_in)
{
  p_out = p_in;
  p_out.header = t_in.header;
  p_out.data.resize(p_in.data.size());
  Eigen::Transform<float,3,Eigen::Affine> t = Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
                                                                   t_in.transform.translation.z) * Eigen::Quaternion<float>(
                                                                     t_in.transform.rotation.w, t_in.transform.rotation.x,
                                                                     t_in.transform.rotation.y, t_in.transform.rotation.z);

  Eigen::Map<Eigen::Matrix3Xf, 0, Eigen::Stride<4, 1> > mapped_in ((float*) &p_in.data.front(), 4, p_in.width, Eigen::Stride<4, 1> ());
  Eigen::Map<Eigen::Matrix3Xf, 0, Eigen::Stride<4, 1> > mapped_out ((float*) &p_out.data.front(), 4, p_out.width, Eigen::Stride<4, 1> ());

  mapped_out = t * mapped_in;
}

void ground_removal(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > >& pclouds) {
    auto in = pclouds->back();
    pclouds->push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));
    auto out = pclouds->back();

    pcl::PassThrough<pcl::PointXYZ> pass;
    // process only points near ground
    pass.setInputCloud (in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-INFINITY, 0.3);
    pcl::IndicesPtr z_indices (new std::vector<int>);
    pass.filter (*z_indices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(reconf.sac_threshold);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(pcl::deg2rad(float(reconf.sac_eps_angle)));
    seg.setInputCloud(in);
    seg.setIndices(z_indices);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Extract the inliers
    extract.setInputCloud (in);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*out);
}

void voxel_grid(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > >& pclouds) {
    auto in = pclouds->back();
    pclouds->push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));
    auto out = pclouds->back();

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(in);
    auto ls = reconf.leaf_size;
    voxel_grid.setLeafSize(ls, ls, ls);
    voxel_grid.filter(*out);
}

void pass_through(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > >& pclouds) {
    auto in = pclouds->back();
    pclouds->push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));
    auto out = pclouds->back();

    // Passthrough filtering for x and z
    pcl::PassThrough<pcl::PointXYZ> pass;
    // x pass
    pass.setInputCloud (in);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 5.0);
    pcl::IndicesPtr pass_x_indices (new std::vector<int>);
    pass.filter (*pass_x_indices);
    // z pass
    pass.setIndices(pass_x_indices);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (reconf.z_min, reconf.z_max);
    pass.filter (*out);
}

void shadow_points(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > >& pclouds) {
    auto in = pclouds->back();
    pclouds->push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));
    auto out = pclouds->back();

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(in);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch (reconf.normal_radius);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    normal_estimation.compute(*cloud_normals);

    pcl::ShadowPoints<pcl::PointXYZ, pcl::Normal> shadow_removal;
    shadow_removal.setInputCloud(in);
    shadow_removal.setNormals(cloud_normals);
    shadow_removal.setThreshold(reconf.shadow_threshold);

    shadow_removal.filter(*out);
}

void projection(boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > >& pclouds) {
    auto in = pclouds->back();
    pclouds->push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));
    auto out = pclouds->back();

    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0;
    // Create the filtering object
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (in);
    proj.setModelCoefficients (coefficients);
    // Create output cloud and apply projection
    proj.filter(*out);
}

void euclidean_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& in, boost::shared_ptr<std::vector<pcl::PointIndices> >& out){
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(in);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
    clustering.setClusterTolerance(reconf.cluster_tolerance);
    clustering.setMinClusterSize(reconf.min_cluster_size);
    clustering.setMaxClusterSize(reconf.max_cluster_size);
    clustering.setSearchMethod(tree);
    clustering.setInputCloud(in);
    clustering.extract(*out);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
{
    // Affine transformation of the point cloud
    sensor_msgs::PointCloud2Ptr transformed_cloud (new sensor_msgs::PointCloud2);
    doTransform(*input_cloud, *transformed_cloud, static_transform);

    // Vector of all pcl point clouds
    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > pclouds;
    boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > > pclouds (new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >);

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pclouds->push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));    
    pcl::fromROSMsg(*transformed_cloud, *pclouds->back());

    if (reconf.enable_voxel_grid){
        voxel_grid(pclouds);
        grid_pub.publish(pclouds->back());
    }

    if (reconf.enable_ground_removal){
        ground_removal(pclouds);
    }

    if (reconf.enable_passthrough){
        pass_through(pclouds);
        pass_pub.publish(pclouds->back());
    }

    if (reconf.enable_shadow_filtering){
        shadow_points(pclouds);
        noshadow_pub.publish(pclouds->back());
    }

    if (reconf.enable_projection){
        projection(pclouds);
        proj_pub.publish(pclouds->back());
    }

    if (reconf.enable_clustering){
        boost::shared_ptr<std::vector<pcl::PointIndices> > cluster_indices (new std::vector<pcl::PointIndices>);
        euclidean_clustering(pclouds->back(), cluster_indices);

        setMarker({0, 0, 0}, -1, visualization_msgs::Marker::DELETEALL);
        int32_t cluster_id = 0;
        // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
        for (std::vector<pcl::PointIndices>::const_iterator cit = cluster_indices->begin (); cit != cluster_indices->end (); ++cit, ++cluster_id)
        {
            Eigen::Vector3f cluster_mean (0, 0, 0);
            // now we are in a vector of indices pertaining to a single cluster.
            // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
            for (std::vector<int>::const_iterator pit = cit->indices.begin (); pit != cit->indices.end (); ++pit)
            {
                cluster_mean += pclouds->back()->points[*pit].getVector3fMap();
            }
            cluster_mean /= cit->indices.size();

            setMarker({cluster_mean.x(), cluster_mean.y(), 0}, cluster_id);
        }
    }
    else {
        setMarker({0, 0, 0}, -1, visualization_msgs::Marker::DELETEALL);
    }

    final_pub.publish(pclouds->back());
}


int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ground_plane_detection");
    ros::NodeHandle nh;

    std::string base_frame_name = argv[1];

    dynamic_reconfigure::Server<segmenter_reconfig::ObjectSegmenterConfig> server;
    dynamic_reconfigure::Server<segmenter_reconfig::ObjectSegmenterConfig>::CallbackType f;

    f = boost::bind(&reconf_cb, _1, _2);
    server.setCallback(f);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while (nh.ok())
    {
        try
        {
            static_transform = tf_buffer.lookupTransform(base_frame_name, "camera_depth_optical_frame", ros::Time::now(), ros::Duration(3.0));
            break;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

    // plane_pub = nh.advertise<pcl_msgs::ModelCoefficients> ("plane_coeff", 1);
    pass_pub = nh.advertise<sensor_msgs::PointCloud2> ("passed_cloud", 1);
    proj_pub = nh.advertise<sensor_msgs::PointCloud2> ("proj_cloud", 1);
    grid_pub = nh.advertise<sensor_msgs::PointCloud2> ("grid_cloud", 1);
    noshadow_pub = nh.advertise<sensor_msgs::PointCloud2> ("noshadow_cloud", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    final_pub = nh.advertise<sensor_msgs::PointCloud2> ("final_cloud", 1);

	ros::MultiThreadedSpinner spinner(4);

	// ros::Timer monitor_timer = nh.createTimer(ros::Duration(0.02), boost::bind(monitorCallBack, _1, monitorPub));
	// ros::Timer command_timer = nh.createTimer(ros::Duration(0.1), commandCallBack);

	spinner.spin();

    // Spin
    // ros::spin();
}
