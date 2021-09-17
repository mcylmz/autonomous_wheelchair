#ifndef RRTSTAR_PLUGIN_H
#define RRTSTAR_PLUGIN_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf/tf.h>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

#include <string>
#include <vector>

#define INTERP_NUM 100


namespace RRTStarPlugin
{
    class RRTStarPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        RRTStarPlanner();
        RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmapROS);

        // Overridden functions from nav_core::BaseGlobalPlanner
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmapROS);

        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        bool isStateValid(const ompl::base::SpaceInformation *si,
                          const ompl::base::State *state);
    private:
        ros::NodeHandle nodeHandle_;
        costmap_2d::Costmap2DROS *costmapROS_;
        costmap_2d::Costmap2D *costmap_;
        bool initialized_;
        unsigned int mapSizeInCellsX_;
        unsigned int mapSizeInCellsY_;
        float xStart_;
        float yStart_;
        float startYaw_;
        float xGoal_;
        float yGoal_;
        float goalYaw_;
    };
}

#endif