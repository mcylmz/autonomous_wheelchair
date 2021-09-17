#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Eigen>
#include <cmath>

int n_duplicates;

void inflate(const sensor_msgs::LaserScanConstPtr &in, const sensor_msgs::LaserScanPtr &out, float radius)
{
    using Vec2 = Eigen::Vector2f;

    // Calculating coordinates of the points from the input and filling the output
    float angle = in->angle_min;
    for (auto range_it = in->ranges.begin(); range_it != in->ranges.end(); ++range_it, (angle += in->angle_increment))
    {
        // Do not inflate max range
        if (*range_it >= in->range_max)
            continue;

        // Minimum inflation range is radius
        float range;
        if (*range_it < radius)
            range = radius;
        else
            range = *range_it;

        Vec2 pt(range * cosf(angle), range * sinf(angle));

        const float angular_increment = M_PI / n_duplicates;

        // Trace inflation half circle
        float trace_begin = angle + M_PI_2;
        for (int trace_idx = 0; trace_idx <= n_duplicates; ++trace_idx)
        {
            float theta = trace_begin + trace_idx * angular_increment;

            Vec2 inf_pt = pt + Vec2(cosf(theta) * radius, sinf(theta) * radius);

            float distance = inf_pt.norm();
            float angle = atan2f(inf_pt.y(), inf_pt.x());

            std::size_t inx = roundf((angle - out->angle_min) / out->angle_increment);

            if (distance < out->ranges[inx])
                out->ranges[inx] = distance;
        }
    }
}

ros::Publisher scan_pub;
float radius;

void scan_cb(const sensor_msgs::LaserScanConstPtr &input_scan)
{
    sensor_msgs::LaserScanPtr output_scan(new sensor_msgs::LaserScan);

    output_scan->header.stamp = ros::Time::now();
    output_scan->header.frame_id = input_scan->header.frame_id;
    output_scan->angle_min = input_scan->angle_min;
    output_scan->angle_max = input_scan->angle_max;
    output_scan->angle_increment = input_scan->angle_increment;
    output_scan->time_increment = input_scan->time_increment;
    output_scan->scan_time = input_scan->scan_time;
    output_scan->range_min = input_scan->range_min;
    output_scan->range_max = input_scan->range_max;
    int num_readings = ((output_scan->angle_max - output_scan->angle_min) / output_scan->angle_increment) + 1; 
    output_scan->ranges.resize(num_readings);
    std::fill(output_scan->ranges.begin(), output_scan->ranges.end(), output_scan->range_max);

    inflate(input_scan, output_scan, radius);

    scan_pub.publish(output_scan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_inflater");
    ros::NodeHandle node_handle;

    if (argc != 3)
    {
        ROS_ERROR("Invalid number of parameters\n"
                  "usage: laser_inflater <radius> <n_duplicates>");
        return -1;
    }

    radius = atof(argv[1]);
    n_duplicates = atoi(argv[2]);
    ros::Subscriber scan_sub = node_handle.subscribe("input", 1, scan_cb);
    scan_pub = node_handle.advertise<sensor_msgs::LaserScan>("output", 1);   

    ros::spin();
    return 0;
};