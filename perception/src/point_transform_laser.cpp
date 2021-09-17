#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <math.h>

struct globals
{
    std::string base_frame;
    std::string lidar_frame;

    float angle_min = -M_PI_4 * 0.93;
    float angle_max = M_PI_4 * 0.93;
    int num_readings = 91;
    float angle_increment = (angle_max - angle_min) / (num_readings - 1);

    float range_max = 5;
    int laser_hz = 30;

    Eigen::Vector3f box_min = {0, -5, 0.3};
    Eigen::Vector3f box_max = {5, 5, 1.3};

    geometry_msgs::TransformStamped camera_transform;
    geometry_msgs::TransformStamped lidar_transform;
} globals;


ros::Publisher scan_pub;

void cloudToScan(const sensor_msgs::PointCloud2 &p_in,
           const geometry_msgs::TransformStamped &t_in,
           const Eigen::Vector3f &box_min,
           const Eigen::Vector3f &box_max,
           sensor_msgs::LaserScan &scan,
           const Eigen::Vector2f &scan_axis)
{
    Eigen::Transform<float, 3, Eigen::Affine> t = Eigen::Translation3f(t_in.transform.translation.x, t_in.transform.translation.y,
                                                                       t_in.transform.translation.z) *
                                                  Eigen::Quaternion<float>(
                                                      t_in.transform.rotation.w, t_in.transform.rotation.x,
                                                      t_in.transform.rotation.y, t_in.transform.rotation.z);

    Eigen::Map<Eigen::Matrix4Xf, 0, Eigen::Stride<4, 1> > mapped_in ((float*) &p_in.data.front(), 4, p_in.width, Eigen::Stride<4, 1> ());

    for (std::size_t col_inx = 0; col_inx < mapped_in.cols(); ++col_inx)
    {
        Eigen::Vector3f point = t * mapped_in.col(col_inx).head<3>(3);

        // If the point is in the box
        if ((box_min.z() <= point.z()) && (point.z() <= box_max.z()) && // Cut top and bottom
            (point.x() >= scan_axis.x())) // Get only the points in front of the lidar
        {
            auto delta = point.head(2) - scan_axis;
            float distance = delta.norm(); // Be careful! Use only x,y for normal calculation

            if ( (scan.range_min <= distance) && (distance <= scan.range_max) )
            {
                float angle = atan2f(delta.y(), delta.x());
                // Not checking angle boundaries or ranges index might cause segmentation fault
                if ((scan.angle_min <= angle) && (angle <= scan.angle_max))
                {
                    std::size_t inx = roundf((angle - scan.angle_min) / scan.angle_increment);
                    
                    if (distance < scan.ranges[inx])
                        scan.ranges[inx] = distance;
                }
            }
        }
    }
}


void trans_laser_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
{
    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan);

    // Assuming that the camera_frame is just over base_frame
    auto translation = globals.lidar_transform.transform.translation;
    Eigen::Vector2f scan_axis(translation.x, translation.y);

    scan->header.stamp = ros::Time::now();
    scan->header.frame_id = globals.lidar_frame.c_str();
    scan->angle_min = globals.angle_min;
    scan->angle_max = globals.angle_max;
    scan->angle_increment = globals.angle_increment;
    scan->time_increment = 0;
    scan->scan_time = (1.0 / globals.laser_hz);
    scan->range_min = 0.0;
    scan->range_max = globals.range_max;
    scan->ranges.resize(globals.num_readings);
    scan->ranges.assign(globals.num_readings, globals.range_max);

    cloudToScan(*input_cloud, globals.camera_transform, globals.box_min, globals.box_max, *scan, scan_axis);

    scan_pub.publish(scan);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "point_transform_laser");
    ros::NodeHandle nh;

    if (argc != 4)
    {
        ROS_ERROR("Invalid number of parameters\n"
                  "usage: point_transform_laser <base_frame> <lidar_frame> <range_max>");
        return -1;
    }

    globals.base_frame = argv[1];
    globals.lidar_frame = argv[2];
    globals.range_max = atof(argv[3]);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    while (nh.ok())
    {
        try {
            globals.camera_transform = tf_buffer.lookupTransform(globals.base_frame, "camera_depth_optical_frame", ros::Time::now(), ros::Duration(3.0));
            break;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    while (nh.ok())
    {
        try {
            globals.lidar_transform = tf_buffer.lookupTransform(globals.base_frame, globals.lidar_frame, ros::Time::now(), ros::Duration(3.0));
            break;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

    scan_pub = nh.advertise<sensor_msgs::LaserScan>("pseudo_scan", 1);

    ros::Subscriber input_sub = nh.subscribe("input", 1, trans_laser_cb);

    ros::MultiThreadedSpinner spinner(4);

    // ros::Timer monitor_timer = nh.createTimer(ros::Duration(0.02), boost::bind(monitorCallBack, _1, monitorPub));
    // ros::Timer command_timer = nh.createTimer(ros::Duration(0.1), commandCallBack);

    spinner.spin();
}
