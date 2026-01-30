#include <pluginlib/class_list_macros.h>
#include <fgm_plugin/fgm_plugin.h>

// Register this planner as a BaseLocalPlannerPlugin
PLUGINLIB_EXPORT_CLASS(fgm_plugin::FGMPlannerROS, nav_core::BaseLocalPlanner)

namespace fgm_plugin
{
    FGMPlannerROS::FGMPlannerROS()
        : initialized_(false), goalDistTolerance_(1), totalError_(0.0), theta_(0.0),
            integralError_(0.0), errorOld_(0.0), sampleTime_(0.0), calcLastCBTime_(false),
            isGapExist_(true), goalReached_(false), dataTimeoutThreshold_(0.5),
            lookAheadDist_(2.75) {
        // Initialize timestamps to current time
        lastPoseTime_ = ros::Time::now();
        lastLaserTime_ = ros::Time::now();
        lastVelTime_ = ros::Time::now();
    }


    void FGMPlannerROS::initialize(std::string name, tf::TransformListener* tf,
                                costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_) {
            ros::NodeHandle nh("~/fgm_plugin");
            nh_ = nh;
        
            tf_ = tf;
            costmapROS_ = costmap_ros;
            costmapROS_->getRobotPose(currentPoseTF_);
            costmap_ = costmapROS_->getCostmap();

            // Publishers - use relative names for better composability
            globalPlanPub_ = nh_.advertise<nav_msgs::Path>("rrtstar_global_plan", 1);
            distToGoalPub_ = nh_.advertise<std_msgs::Float32>("distance_to_goal", 1);
            wRefPub_ = nh_.advertise<std_msgs::Float32>("angular_vel_output", 1);
            markerPub_ = nh_.advertise<visualization_msgs::Marker>("gap_center_marker", 1);

            // Subscribers - use relative names to allow remapping in launch files
            scaledLinVelSub_ = nh_.subscribe("scaled_lin_vel", 100, &FGMPlannerROS::scaledLinVelCallback, this);
            odomSub_ = nh_.subscribe("odom", 100, &FGMPlannerROS::odomCallback, this);
            laserScanSub_ = nh_.subscribe("inflated_pseudo_scan", 100, &FGMPlannerROS::laserScanCallback, this);
            poseSub_ = nh_.subscribe("amcl_pose", 100, &FGMPlannerROS::poseCallback, this);

            // Load parameters with validation
            nh_.param("data_timeout_threshold", dataTimeoutThreshold_, 0.5);

            if (!nh_.getParam("/move_base/FGMPlanner/look_ahead_dist", lookAheadDist_)) {
                lookAheadDist_ = 2.75;  // Default value
                ROS_WARN("FGM: look_ahead_dist parameter not found, using default: %.2f", lookAheadDist_);
            }

            // Validate look_ahead_dist
            if (lookAheadDist_ <= 0.0 || lookAheadDist_ > 10.0) {
                ROS_ERROR("FGM: Invalid look_ahead_dist value: %.2f (must be in range (0, 10])", lookAheadDist_);
                throw std::runtime_error("Invalid look_ahead_dist parameter");
            }

            // Heading controller settings - P = 0.31 is working well
            headingController_.setPIDCoeffs(0.75, 0.0, 0.0);
            headingController_.setDerivativeFilterConstant(50.0);
            headingController_.setOutputSaturations(-0.940, 0.940);

            ROS_INFO("FGM planner has been initialized successfully.");
            initialized_ = true;
        }
        else {
            ROS_WARN("FGM has already been initialized, doing nothing.");
        }
        
    } // end function initialize


    bool FGMPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
        if (!initialized_) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        globalPlan_.clear();
        globalPlan_ = orig_global_plan;
        goalReached_ = false;
        
        ROS_INFO("Got new plan");
        
        return true;
    } // end function setPlan


    bool FGMPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        ROS_INFO_ONCE("Computing velocity commands...");

        // Validate sensor data before use
        if (!isDataValid()) {
            ROS_ERROR_THROTTLE(1.0, "FGM: Cannot compute velocity - invalid or stale sensor data");
            // Return zero velocity for safety
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;
            return false;
        }

        // Publish global plan for visualization
        publishGlobalPlan(globalPlan_);

        // Find a local goal in the global plan considering look ahead distance
        // Safe to dereference pointers now after validation
        for (unsigned int i = 0; i < globalPlan_.size(); i++) {
            currentPose_.position.x = posePtr_->pose.pose.position.x;
            currentPose_.position.y = posePtr_->pose.pose.position.y;
            currentPose_.position.z = posePtr_->pose.pose.position.z;

            double waypointX = globalPlan_[i].pose.position.x;
            double waypointY = globalPlan_[i].pose.position.y;

            double diffX = waypointX - currentPose_.position.x;
            double diffY = waypointY - currentPose_.position.y;

            if (hypot(diffX, diffY) > lookAheadDist_ && distanceToGlobalGoal() > goalDistTolerance_) {
                currentGoalPoseIdx_ = i;
                break;
            }
            else if (distanceToGlobalGoal() < goalDistTolerance_) {
                currentGoalPoseIdx_ = globalPlan_.size() - 1;
                break;
            }
            else {
                continue;
            }
        }

        // ROS_INFO_STREAM("Current goal index: " << currentGoalPoseIdx_);
        
        currentGoalPose_ = globalPlan_.at(currentGoalPoseIdx_).pose;
        // ROS_WARN_STREAM("Goal: " << currentGoalPose_.position.x);

        if (distanceToGlobalGoal() < goalDistTolerance_) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0;

            goalReached_ = true;
        }
        else {
            double phiFinal = FGMCallback(); // FGM Algorithm
            headingController_.setSampleTime(ros::Time::now().toSec() - lastCallbackTime_);
            // ROS_WARN_STREAM("Sample time: " << headingController_.getSampleTime());
            double omega = headingController_.derivativeFilteredControlSignal(phiFinal);

            // Print and publish the distance to global goal
            double distToGlobGoal = distanceToGlobalGoal();
            // ROS_INFO_STREAM("Distance to global goal: " << distToGlobGoal);
            publishDistToGoal(distToGoalPub_, distToGlobGoal);

            // Publish reference omega for Fuzzy planner
            publishWRef(wRefPub_, omega);

            // Linear velocity value calculated by the Fuzzy velocity planner
            double linearVel = scaledLinVelPtr_->data;

            // Send velocity commands to robot's base
            // cmd_vel.linear.x = 1.25;
            cmd_vel.linear.x = linearVel;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;

            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = omega;
        }
        
        return true;
    } // end function computeVelocityCommands


    bool FGMPlannerROS::isGoalReached() {
        if (!initialized_) {
            ROS_ERROR("FGM planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if (!costmapROS_->getRobotPose(currentPoseTF_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if (goalReached_) {
            ROS_INFO("Goal reached!");
            return true;
        }

        return false;
    } // end function isGoalReached


    void FGMPlannerROS::odomCallback(boost::shared_ptr<nav_msgs::Odometry const> msg) {
        odomPtr_ = msg;
    } // end function odomCallback


    void FGMPlannerROS::laserScanCallback(boost::shared_ptr<sensor_msgs::LaserScan const> msg) {
        laserScanPtr_ = msg;
        lastLaserTime_ = ros::Time::now();
    } // end function laserScanCallback


    void FGMPlannerROS::poseCallback(boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> msg) {
        posePtr_ = msg;
        lastPoseTime_ = ros::Time::now();
    } // end function poseCallback
    

    double FGMPlannerROS::distanceToGlobalGoal() {
        // Defensive check - should not be called if data is invalid
        if (!posePtr_) {
            ROS_ERROR_THROTTLE(1.0, "FGM: distanceToGlobalGoal called with null pose pointer");
            return std::numeric_limits<double>::max();
        }

        currentPose_.position.x = posePtr_->pose.pose.position.x;
        currentPose_.position.y = posePtr_->pose.pose.position.y;
        currentPose_.position.z = posePtr_->pose.pose.position.z;

        geometry_msgs::PoseStamped globalGoal = globalPlan_.back();
        double xDist = globalGoal.pose.position.x - currentPose_.position.x;
        double yDist = globalGoal.pose.position.y - currentPose_.position.y;

        return hypot(xDist, yDist);
    }


    double FGMPlannerROS::FGMCallback() {
        ROS_INFO_STREAM("FGM started");
        lastCallbackTime_ = ros::Time::now().toSec();

        // Defensive check - this should not be called if data is invalid
        // but we add this safety check anyway
        if (!posePtr_ || !laserScanPtr_) {
            ROS_ERROR("FGM: FGMCallback called with null pointers!");
            return 0.0;  // Return zero angular velocity
        }

        // Get odometry informations
        // WARNING: These are not odometry information! Variable names remained
        // unchanged since the latest update. These are AMCL positions.
        double odomRX = posePtr_->pose.pose.position.x;
        double odomRY = posePtr_->pose.pose.position.y;
        currentPose_.orientation = posePtr_->pose.pose.orientation;
        double odomRYaw = tf::getYaw(currentPose_.orientation);

        double goalX;
        double goalY;
        // if (isGapExist_) {
        //     goalX = currentGoalPose_.position.x;
        //     goalY = currentGoalPose_.position.y;
        // }
        // else {
        //     goalX = gapPointMapFrame_.point.x;
        //     goalY = gapPointMapFrame_.point.y;
        // }
        goalX = currentGoalPose_.position.x;
        goalY = currentGoalPose_.position.y;

        // Calculate robot's goal angle
        double phiGoalConstraint = atan2(goalY - odomRY, goalX - odomRX);

        // Calculate goal phi
        double phiGoal;
        if (phiGoalConstraint - odomRYaw > M_PI)
            phiGoal = (phiGoalConstraint - odomRYaw) - 2 * M_PI;
        else if (phiGoalConstraint - odomRYaw < -M_PI)
            phiGoal = (phiGoalConstraint - odomRYaw) + 2 * M_PI;
        else
            phiGoal = phiGoalConstraint - odomRYaw;
                
        // Calculate surrounding gaps
        std::deque<double> obstacleAngles;
        std::deque<unsigned int> obstacleAnglesIdx;

        obstacleAngles.push_back(laserScanPtr_->angle_min);
        obstacleAnglesIdx.push_back(0);
        int obstacleCounter = 1;

        // Get laser ranges
        std::vector<double> laserRanges;
        for (unsigned int i = 0; i < laserScanPtr_->ranges.size(); i++) {
            // if (isinff(laserScanPtr_->ranges[i]))
            //     laserRanges.push_back(laserScanPtr_->range_max + 99);
            // else
            //     laserRanges.push_back(laserScanPtr_->ranges[i]);

            if (fabs(laserScanPtr_->ranges[i] - 2.75) < 0.01) // Detect infinity
                laserRanges.push_back(laserScanPtr_->range_max + 99.);
            else
                laserRanges.push_back(laserScanPtr_->ranges[i]); // In-range measurement

        }

        // Obstacle detection using laser measurements
        for (unsigned int i = 0; i < laserRanges.size() - 1; i++) {
            int temp;
            double tempAngle;
            if (fabs(laserRanges[i] - laserRanges[i+1]) > (laserScanPtr_->range_max) + 0.1) {
                if (i < floor(laserRanges.size() / 2)) {
                    // Right side of laser readings
                    if (laserRanges[i] > laserRanges[i+1])
                        temp = i + 1;
                    else
                        temp = i;
                    
                    tempAngle = -(laserScanPtr_->angle_max - (laserScanPtr_->angle_increment * temp));
                }
                else {
                    if (i == floor(laserRanges.size() / 2)) {
                        // Front of the robot
                        temp = i;
                        tempAngle = 0;
                    }
                    else {
                        // Left side of laser readings
                        if (laserRanges[i] > laserRanges[i+1])
                            temp = i + 1;
                        else
                            temp = i;                       

                        tempAngle = ((laserScanPtr_->angle_increment * temp) + laserScanPtr_->angle_min);
                    }
                }

                obstacleAngles.push_back(tempAngle);
                obstacleAnglesIdx.push_back(temp);
                obstacleCounter++;
            }

        } // end for

        obstacleAngles.push_back(laserScanPtr_->angle_max);
        obstacleAnglesIdx.push_back(laserRanges.size() - 1);


        // Check borders of the gaps
        if (laserRanges.front() > 0 && (laserRanges.front() != 101.75)) {
            obstacleAngles.pop_front();
            obstacleAnglesIdx.pop_front();
            obstacleCounter--;
        }

        if (laserRanges.back() > 0 && (laserRanges.back() != 101.75)) {
            obstacleAngles.pop_back();
            obstacleAnglesIdx.pop_back();
            obstacleCounter--;
        }

        ROS_WARN_STREAM("Obstacle angles:");
        for (auto item : obstacleAngles) {
            ROS_WARN_STREAM(item);
        }

        
        // Gap calculations
        std::vector<double> gap;
        double tempTheta;
        for (unsigned int i = 1; i < obstacleAngles.size(); i = i + 2) {
            tempTheta = fabs(obstacleAngles[i] - obstacleAngles[i-1]);
            gap.push_back(tempTheta);
        }
        
        ROS_WARN_STREAM("Gap angles: ");
        for (const double& g : gap) {
            ROS_WARN_STREAM(g);
        }


        int maxGapIndex = std::max_element(gap.begin(), gap.end()) - gap.begin();

        double theta;
        int angleIndex;
        double phiGapCenter;
        double phiFinal;
        double alpha = 20.0;
        auto dminIdxItr = std::min_element(gap.begin(), gap.end());
        int dminIdx = std::distance(gap.begin(), dminIdxItr);
        double dmin = laserRanges.at(dminIdx);
        double beta = 3.0; // 1

        if (!gap.empty()) {
            isGapExist_ = true;
            angleIndex = maxGapIndex * 2;
            double d1 = laserRanges[obstacleAnglesIdx[angleIndex]];
            double d2 = laserRanges[obstacleAnglesIdx[angleIndex+1]];
            double phi1 = obstacleAngles[angleIndex];
            double phi2 = obstacleAngles[angleIndex+1];

            double numer = d1 + d2 * cos(phi1 - phi2);
            double denum = sqrt(pow(d1, 2) + pow(d2, 2) + 2 * d1 * d2 * cos(phi1 - phi2));

            phiGapCenter = acos(numer / denum) + phi1;
            // phiGapCenter = (phi1 + phi2) / 2.0;
            phiFinal = (((alpha / dmin) * phiGapCenter) + (beta * phiGoal)) / (alpha / dmin + beta);

            double tempPhiFinal = fmod(phiFinal, 2 * M_PI);

            if (tempPhiFinal > M_PI)
                phiFinal = tempPhiFinal - 2 * M_PI;
            else if (tempPhiFinal < -M_PI)
                 phiFinal = tempPhiFinal + 2 * M_PI;
            else
                 phiFinal = tempPhiFinal;

            // ROS_INFO_STREAM("Final phi: " << phiFinal);
            ROS_INFO_STREAM("Gap center phi: " << phiGapCenter);
            // ROS_INFO_STREAM("Goal phi: " << phiGoal);


            if (gap.size() != 0) {
                ROS_INFO_STREAM("Updating the last seen gap.");
                // Create a gap center point
                double h_squared = (pow(d1, 2.0) + pow(d2, 2.0) + 2 * d1 * d2 * cos(phi1 + phi2)) / 4.0;
                double h = sqrt(h_squared);
                
                geometry_msgs::PointStamped gapPointRobotFrame;
                gapPointRobotFrame.header.frame_id = "base_footprint";
                gapPointRobotFrame.header.stamp = ros::Time();

                // Gap point coordinates defined in the robot's frame
                gapPointRobotFrame.point.x = h * cos(phiGapCenter);
                gapPointRobotFrame.point.y = h * sin(phiGapCenter);
                gapPointRobotFrame.point.z = 0.0;

                try {
                    geometry_msgs::PointStamped gapPointMapFrame;
                    tf_->transformPoint("map", gapPointRobotFrame, gapPointMapFrame);

                    // ROS_INFO_STREAM("Gap robot: " << gapPointRobotFrame.point.x << ", " << gapPointRobotFrame.point.y);
                    // ROS_INFO_STREAM("Gap map: " << gapPointMapFrame.point.x << ", " << gapPointMapFrame.point.y);

                    gapPointMapFrame_ = gapPointMapFrame;

                }
                catch (tf::TransformException& e) {
                    ROS_ERROR("Received an exception trying to transform a point \
                        from \"base_footprint\" to \"map\": %s", e.what());
                }
            }

            visualizeGapCenter();

            return phiFinal;
        }
        else {
            ROS_WARN_STREAM("No gap exists");
            isGapExist_ = false;

            // Remember the last gap center
            odomRX = posePtr_->pose.pose.position.x;
            odomRY = posePtr_->pose.pose.position.y;
            currentPose_.orientation = posePtr_->pose.pose.orientation;
            double odomRYaw = tf::getYaw(currentPose_.orientation);

            double gapPointAngle = atan2(gapPointMapFrame_.point.y, gapPointMapFrame_.point.x);
            phiGapCenter = gapPointAngle - odomRYaw;            
            
            phiFinal = (((alpha / dmin) * phiGapCenter) + (beta * phiGoal)) / (alpha / dmin + beta);

            double tempPhiFinal = fmod(phiFinal, 2 * M_PI);

            if (tempPhiFinal > M_PI)
                phiFinal = tempPhiFinal - 2 * M_PI;
            else if (tempPhiFinal < -M_PI)
                 phiFinal = tempPhiFinal + 2 * M_PI;
            else
                 phiFinal = tempPhiFinal;

            visualizeGapCenter();

            ROS_WARN_STREAM("No gap phi gap center: " << phiGapCenter);
            return phiFinal;
        }

        // ROS_WARN_STREAM("Phi final: " << phiFinal);
        // return phiFinal;
    } // end function FGMCallback

    void FGMPlannerROS::scaledLinVelCallback(boost::shared_ptr<std_msgs::Float32 const> msg) {
        scaledLinVelPtr_ = msg;
        lastVelTime_ = ros::Time::now();
    } // end function scaledLinVelCallback

    bool FGMPlannerROS::isDataValid() const {
        // Check for null pointers
        if (!posePtr_ || !laserScanPtr_ || !scaledLinVelPtr_) {
            ROS_ERROR_THROTTLE(1.0, "FGM: Missing sensor data (pose: %s, laser: %s, vel: %s)",
                             posePtr_ ? "OK" : "NULL",
                             laserScanPtr_ ? "OK" : "NULL",
                             scaledLinVelPtr_ ? "OK" : "NULL");
            return false;
        }

        // Check for stale data
        ros::Time now = ros::Time::now();

        if ((now - lastPoseTime_).toSec() > dataTimeoutThreshold_) {
            ROS_WARN_THROTTLE(1.0, "FGM: Pose data is stale (%.2f s old)",
                            (now - lastPoseTime_).toSec());
            return false;
        }

        if ((now - lastLaserTime_).toSec() > dataTimeoutThreshold_) {
            ROS_WARN_THROTTLE(1.0, "FGM: Laser data is stale (%.2f s old)",
                            (now - lastLaserTime_).toSec());
            return false;
        }

        if ((now - lastVelTime_).toSec() > dataTimeoutThreshold_) {
            ROS_WARN_THROTTLE(1.0, "FGM: Velocity data is stale (%.2f s old)",
                            (now - lastVelTime_).toSec());
            return false;
        }

        return true;
    } // end function isDataValid

    void FGMPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
        base_local_planner::publishPlan(path, globalPlanPub_);
    } // end function publishGlobalPlan

    void FGMPlannerROS::visualizeGapCenter() {
        uint32_t shape = visualization_msgs::Marker::SPHERE;
        visualization_msgs::Marker gapMarker;
        gapMarker.header.frame_id = "map";
        gapMarker.header.stamp = ros::Time::now();
        gapMarker.ns = "gap_point";
        gapMarker.id = 0;
        gapMarker.type = shape;
        gapMarker.action = visualization_msgs::Marker::ADD;

        gapMarker.pose.position.x = gapPointMapFrame_.point.x;
        gapMarker.pose.position.y = gapPointMapFrame_.point.y;
        gapMarker.pose.position.z = 0.5;
        gapMarker.pose.orientation.x = 0.0;
        gapMarker.pose.orientation.y = 0.0;
        gapMarker.pose.orientation.z = 0.0;
        gapMarker.pose.orientation.w = 1.0;

        gapMarker.scale.x = 0.2;
        gapMarker.scale.y = 0.2;
        gapMarker.scale.z = 0.2;

        gapMarker.color.b = 1.0f;
        gapMarker.color.a = 1.0;

        markerPub_.publish(gapMarker);
    } // end function visualizeGapCenter
    
    void publishDistToGoal(const ros::Publisher& pub, double dist) {
        std_msgs::Float32 msg;
        msg.data = dist;

        pub.publish(msg);
    } // end function publishDistToGoal

    void publishWRef(const ros::Publisher& pub, double wRef) {
        std_msgs::Float32 msg;
        msg.data = wRef;

        pub.publish(msg);
    } // end function publishWRef

} // end namespace fgm_plugin
