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

            // Load parameters
            loadParameters();

            ROS_INFO("RRT* planner initialized successfully");
            ROS_INFO("  Planning timeout: %.2f s", planningTimeout_);
            ROS_INFO("  Turning radius: %.2f m", turningRadius_);
            ROS_INFO("  Path simplification: %s", pathSimplification_ ? "enabled" : "disabled");
        }
        else
        {
            ROS_WARN("RRT* planner has already been initialized!");
        }

        initialized_ = true;
    } // end function initialize

    void RRTStarPlanner::loadParameters()
    {
        // Planning parameters
        nodeHandle_.param("planning_timeout", planningTimeout_, 5.0);
        nodeHandle_.param("turning_radius", turningRadius_, 0.9);
        nodeHandle_.param("optimization_time", optimizationTime_, 1.0);

        // Path quality parameters
        nodeHandle_.param("min_path_length", minPathLength_, 0.5);
        nodeHandle_.param("max_path_length", maxPathLength_, 100.0);
        nodeHandle_.param("path_simplification", pathSimplification_, true);
        nodeHandle_.param("interpolation_num", interpolationNum_, INTERP_NUM);

        // Validate parameters
        if (planningTimeout_ <= 0.0) {
            ROS_WARN("Invalid planning_timeout (%.2f), using default: 5.0", planningTimeout_);
            planningTimeout_ = 5.0;
        }

        if (turningRadius_ <= 0.0) {
            ROS_WARN("Invalid turning_radius (%.2f), using default: 0.9", turningRadius_);
            turningRadius_ = 0.9;
        }

        if (minPathLength_ < 0.0 || minPathLength_ >= maxPathLength_) {
            ROS_WARN("Invalid path length limits, using defaults");
            minPathLength_ = 0.5;
            maxPathLength_ = 100.0;
        }

        if (interpolationNum_ < 2) {
            ROS_WARN("Invalid interpolation_num (%d), using default: %d", interpolationNum_, INTERP_NUM);
            interpolationNum_ = INTERP_NUM;
        }
    } // end function loadParameters

    bool RRTStarPlanner::isPathValid(const ompl::geometric::PathGeometric& path) const
    {
        double pathLength = path.length();

        // Check path length bounds
        if (pathLength < minPathLength_) {
            ROS_ERROR("RRT*: Path too short (%.2fm < %.2fm)", pathLength, minPathLength_);
            return false;
        }

        if (pathLength > maxPathLength_) {
            ROS_ERROR("RRT*: Path too long (%.2fm > %.2fm)", pathLength, maxPathLength_);
            return false;
        }

        // Check state count
        if (path.getStateCount() < 2) {
            ROS_ERROR("RRT*: Path has insufficient states (%u)", path.getStateCount());
            return false;
        }

        return true;
    } // end function isPathValid


    bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("RRT planner has not been initialized, use initialize()");
            return false;
        }
        
        ROS_INFO("RRT*: Starting path planning...");

        // Get start and goal positions
        xStart_ = start.pose.position.x;
        yStart_ = start.pose.position.y;
        startYaw_ = tf::getYaw(start.pose.orientation);

        xGoal_ = goal.pose.position.x;
        yGoal_ = goal.pose.position.y;
        goalYaw_ = tf::getYaw(goal.pose.orientation);

        ROS_DEBUG("RRT*: Start (%.2f, %.2f, %.2f), Goal (%.2f, %.2f, %.2f)",
                  xStart_, yStart_, startYaw_, xGoal_, yGoal_, goalYaw_);

        // Create Dubins State Space with configurable turning radius
        ompl::base::StateSpacePtr space(std::make_shared<ompl::base::DubinsStateSpace>(turningRadius_, true));

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

        // Set up the planner
        ss.setup();

        // Attempt to solve the problem within specified timeout
        ROS_DEBUG("RRT*: Planning with %.2fs timeout...", planningTimeout_);
        ompl::base::PlannerStatus solved = ss.solve(planningTimeout_);

        // Check planning result
        if (!solved)
        {
            ROS_ERROR("RRT*: Planning failed (status: %s)", solved.asString().c_str());
            return false;
        }

        ROS_INFO("RRT*: Planning succeeded, processing path...");

        // Get the solution path
        ompl::geometric::PathGeometric path = ss.getSolutionPath();

        // Validate path before simplification
        if (!isPathValid(path))
        {
            ROS_ERROR("RRT*: Planned path failed validation");
            return false;
        }

        // Simplify solution if enabled
        if (pathSimplification_)
        {
            ROS_DEBUG("RRT*: Simplifying path...");
            ss.simplifySolution(optimizationTime_);
            path = ss.getSolutionPath();

            // Validate again after simplification
            if (!isPathValid(path))
            {
                ROS_ERROR("RRT*: Simplified path failed validation");
                return false;
            }
        }

        // Interpolate path
        path.interpolate(interpolationNum_);
        double pathLength = path.length();

        ROS_INFO("RRT*: Path generated with %.2fm length and %u states",
                 pathLength, path.getStateCount());

        // Print path for debugging (optional)
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            path.printAsMatrix(std::cout);
        }

        // Convert OMPL path to ROS plan
        plan.clear();
        plan.push_back(start);

        for (unsigned int i = 1; i < path.getStateCount() - 1; i++)
        {
            ompl::base::State *st = path.getState(i);
            const auto *s = st->as<ompl::base::SE2StateSpace::StateType>();

            geometry_msgs::PoseStamped pos;
            pos.header.stamp = ros::Time::now();
            pos.header.frame_id = costmapROS_->getGlobalFrameID();
            pos.pose.position.x = s->getX();
            pos.pose.position.y = s->getY();
            pos.pose.position.z = 0.0;
            pos.pose.orientation = tf::createQuaternionMsgFromYaw(s->getYaw());

            plan.push_back(pos);
        }

        plan.push_back(goal);

        ROS_INFO("RRT*: Plan generated with %zu waypoints", plan.size());
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