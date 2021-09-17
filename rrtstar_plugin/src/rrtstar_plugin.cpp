#include <pluginlib/class_list_macros.h>
#include <rrtstar_plugin/rrtstar_plugin.h>

// Register as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRTStarPlugin::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace RRTStarPlugin
{
    RRTStarPlanner::RRTStarPlanner()
        : costmapROS_(nullptr), initialized_(false) {}

    RRTStarPlanner::RRTStarPlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmapROS)
        : costmapROS_(costmapROS)
    {
        initialize(name, costmapROS);
    }

    void RRTStarPlanner::initialize(std::string name,
                                    costmap_2d::Costmap2DROS* costmapROS)
    {
        if (!initialized_)
        {
            // Initialize map
            costmapROS_ = costmapROS;
            costmap_ = costmapROS->getCostmap();

            // Initialize node handler
            ros::NodeHandle nh("~/RRTStarPlugin");
            nodeHandle_ = nh;

            // Get obstacles in the matrix
            mapSizeInCellsX_ = costmap_->getSizeInCellsX();
            mapSizeInCellsY_ = costmap_->getSizeInCellsY();
        }
        else
        {
            ROS_WARN("RRT* planner has not been initialized!");
        }
        
        // Display an info message
        ROS_INFO("RRT* planner initialized");
        initialized_ = true;
    } // end function initialize


    bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("RRT planner has not been initialized, use initialize()");
            return false;
        }
        
        ROS_INFO("Making plan...");
        // Get start and goal positions
        xStart_ = start.pose.position.x;
        yStart_ = start.pose.position.y;
        startYaw_ = tf::getYaw(start.pose.orientation);

        xGoal_ = goal.pose.position.x;
        yGoal_ = goal.pose.position.y;
        goalYaw_ = tf::getYaw(goal.pose.orientation);
        
        // Create Dubins State Space
        ompl::base::StateSpacePtr space(std::make_shared<ompl::base::DubinsStateSpace>(0.9, true));

        // Set boundaries of the state space
        ompl::base::ScopedState<> start_ompl(space), goal_ompl(space);
        ompl::base::RealVectorBounds bounds(2);

        bounds.setLow(0, costmap_->getOriginX());
        bounds.setLow(1, costmap_->getOriginY());
        bounds.setHigh(0, costmap_->getSizeInMetersX() + costmap_->getOriginX() - 1);
        bounds.setHigh(1, costmap_->getSizeInMetersY() + costmap_->getOriginY() - 1);

        space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

        if (goal.header.frame_id != costmapROS_->getGlobalFrameID())
        {
            ROS_ERROR("This planner will only except goals in the %s frame,"
                        "the goal was sent to the %s frame.",
                        costmapROS_->getGlobalFrameID().c_str(),
                        goal.header.frame_id.c_str());
            return false;
        }

        // Define a SimpleSetup object
        ompl::geometric::SimpleSetup ss(space);

        // Set state validity checker and planner for this space
        const ompl::base::SpaceInformation *si = ss.getSpaceInformation().get();
        ss.setPlanner(std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation()));
        
        ss.setStateValidityChecker([=](const ompl::base::State *state)
            {
                return isStateValid(si, state);
            });

        // Set start and goal states
        start_ompl = {xStart_, yStart_, startYaw_};
        goal_ompl = {xGoal_, yGoal_, goalYaw_};
    
        ss.setStartAndGoalStates(start_ompl, goal_ompl);
   
        // Attemp to solve the problem within specified seconds of planning time
        ompl::base::PlannerStatus solved = ss.solve(2.0); 

        if (solved)
        {
            std::cout << "Found solution: " << std::endl;
            ss.simplifySolution();
            ompl::geometric::PathGeometric path = ss.getSolutionPath();
            path.interpolate(INTERP_NUM);
            double pathLen = path.length();

            // Write the path to standard output
            path.printAsMatrix(std::cout);

            // Store path in C++'s std::vector container
            plan.clear();
            plan.push_back(start);

            for (unsigned int i = 1; i < path.getStateCount()-1; i++)
            {
                ompl::base::State *st = path.getState(i);
                const auto *s = st->as<ompl::base::SE2StateSpace::StateType>();
                
                geometry_msgs::PoseStamped pos;
                pos.pose.position.x = s->getX();
                pos.pose.position.y = s->getY();
                pos.pose.position.z = 0.0;

                pos.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());
                plan.push_back(pos);
            }

            plan.push_back(goal);
        }
        else
        {
            std::cout << "No solution found." << std::endl;
        }

        ROS_WARN("Planning done...");
        return true;
    } // end function makePlan


    bool RRTStarPlanner::isStateValid(const ompl::base::SpaceInformation *si,
                                      const ompl::base::State *state)
    {
        double x, y;
        const auto *s = state->as<ompl::base::SE2StateSpace::StateType>();
        x = s->getX();
        y = s->getY();
        // ROS_WARN_STREAM(x << " " << y);

        double mapResolution = costmap_->getResolution();
        double mapOriginX = costmap_->getOriginX();
        double mapOriginY = costmap_->getOriginY();
        // ROS_WARN_STREAM(mapOriginX << " " << mapOriginY);

        unsigned int xCell, yCell;
        xCell = ((x - mapOriginX) / mapResolution);
        yCell = ((y - mapOriginY) / mapResolution);
        // ROS_WARN_STREAM(xCell << " " << yCell);

        unsigned int cost = costmap_->getCost(xCell, yCell);

        // std::cout << "Cost of (" << xCell << " " << yCell << "): " << cost << std::endl;
        if (cost == 0)
            return true;
        else
            return false;
    } // end function isStateValid
}