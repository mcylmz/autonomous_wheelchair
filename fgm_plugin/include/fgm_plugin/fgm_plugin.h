#ifndef FGM_PLUGIN_H
#define FGM_PLUGIN_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <base_local_planner/goal_functions.h>

#include <vector>
#include <deque>
#include <string>
#include <cmath>
#include <algorithm>
#include <exception>

#include "fgm_plugin/PID.h"


namespace fgm_plugin
{
    class FGMPlannerROS : public nav_core::BaseLocalPlanner
    {
    public:
        FGMPlannerROS();

        // Overridden functions from nav_core::BaseLocalPlanner
        void initialize(std::string name, tf::TransformListener* tf,
                        costmap_2d::Costmap2DROS* costmap_ros);

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        bool isGoalReached();

        // Callbacks
        void odomCallback(boost::shared_ptr<nav_msgs::Odometry const> msg);

        void laserScanCallback(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
        
        void poseCallback(boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> msg);

        double distanceToGlobalGoal();

        double FGMCallback();

        void scaledLinVelCallback(boost::shared_ptr<std_msgs::Float32 const> msg);

        double headingController(double phi);

        double PControl(double phi);

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        void visualizeGapCenter();
        
        ~FGMPlannerROS() {}

    private:
        bool initialized_;
        ros::NodeHandle nh_;
        costmap_2d::Costmap2D* costmap_;
        costmap_2d::Costmap2DROS* costmapROS_;
        tf::TransformListener* tf_;

        bool goalReached_;
        double goalDistTolerance_;
        std::vector<geometry_msgs::PoseStamped> globalPlan_;
        geometry_msgs::Pose currentPose_;
        geometry_msgs::Pose currentGoalPose_;
        tf::Stamped<tf::Pose> currentPoseTF_;

        // Sensor subscriptions
        ros::Subscriber odomSub_;
        ros::Subscriber laserScanSub_;
        ros::Subscriber poseSub_;
        boost::shared_ptr<nav_msgs::Odometry const> odomPtr_;
        boost::shared_ptr<sensor_msgs::LaserScan const> laserScanPtr_;
        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> posePtr_;

        // Publishers
        ros::Publisher globalPlanPub_;
        ros::Publisher distToGoalPub_;
        ros::Publisher wRefPub_;
        ros::Publisher markerPub_;

        // Fuzzy velocity planner subscription
        ros::Subscriber scaledLinVelSub_;
        boost::shared_ptr<std_msgs::Float32 const> scaledLinVelPtr_;

        double lookAheadDist_;
        int currentGoalPoseIdx_;
        bool isGapExist_;

        // Controller parameters
        PIDController headingController_;

        bool calcLastCBTime_;
        double totalError_;
        double theta_;
        double sampleTime_;
        double lastCallbackTime_;
        double integralError_;
        double errorOld_;

        // Last gap center informations
        geometry_msgs::PointStamped gapPointMapFrame_;        

    }; // end class FGMPlannerROS

    // Publisher functions (They are used for fuzzy planners, currently)
    void publishDistToGoal(const ros::Publisher& pub, double dist);

    void publishWRef(const ros::Publisher& pub, double wRef);

} // end namespace fgm_plugin

#endif
