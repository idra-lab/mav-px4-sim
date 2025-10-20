#include <benchmarking/apace/path_smoother.hpp>



namespace benchmarking {
namespace apace {



        void PathSmoother::smoothPath(ompl::geometric::PathGeometric &path)
        {
            // Extract the waypoints from the path
            
            // std::vector<base::State *> waypoints = path.getStates();
            waypoints_.clear();
            times_.clear();

            /* 
            
            x ------ x -------- x
            ^                   ^
            |                   |
            Start                Goal
            x  -o-o- x -o - o-  x
            We start from the first, add nWaypointsPerSegment - 1 to every segment except the last, where we have to keep the last. This is done to avoid repetition.
            */
           
            // Duplicate the first state in the path
            // base::State *start = path.getState(0);
            // base::State *startCopy = si_->cloneState(start);
            // path.getStates().insert(path.getStates().begin(), startCopy);

            
           
            size_t nSegments{path.getStateCount()- 1};
            waypoints_.reserve(nSegments * (numWaypointsPerSegment_-1) +1);


            std::cout << "Smoothing path with " << nSegments << " segments" << std::endl;
            
            double t_i{0}, t_i1{0};
            
           

            Eigen::Matrix3d previousOrientation = Eigen::Matrix3d::Identity(); // Initialize previous orientation
                        
            
            bool polytopesEmpty{freeSpacePolytopes_.empty()}, subMapsEmpty{subMaps_.empty()};
            
            // Iterate through the waypoints and apply DroneOptimizer to each segment
            for (size_t i = 0; i < nSegments; ++i)
            {
                SE3State_ *start = path.getState(i)->as<SE3State_>();
                SE3State_ *goal = path.getState(i+1)->as<SE3State_>();
                
                // Extract start and goal positions and orientations
                Eigen::Vector3d startPos = extractPosition(start);
                Eigen::Matrix3d startOrient = extractOrientation(start).transpose();
                Eigen::Vector3d goalPos = extractPosition(goal);
                Eigen::Matrix3d goalOrient = extractOrientation(goal).transpose();

                Eigen::Quaterniond q_start(startOrient);
                Eigen::Quaterniond q_goal(goalOrient);
                Eigen::Vector3d gravity(0.0, 9.81, 0.0);

                Eigen::Vector3d init_vel(0,0,0), end_vel(0,0,0);
                Eigen::Vector3d init_acc(0,0,0), end_acc(0,0,0);

                Eigen::Matrix3d diff_flat_startOrient(computeAttitudeFromAccelPreserveHeading(q_start, init_acc, gravity));
                Eigen::Matrix3d diff_flat_goalOrient(computeAttitudeFromAccelPreserveHeading(q_goal, end_acc, gravity));



                Eigen::Vector3d segment_dir = goalPos - startPos; // Direction vector from current position to goal position
                segment_dir.normalize(); // Normalize the direction vector

                
                float segmentLength = (goalPos - startPos).norm();
               
                float segmentTime = segmentLength / v_mean;

                if (segmentTime < 0.1)
                {
                    segmentTime = 1; // Ensure a minimum segment time
                }
                t_i1 = t_i+segmentTime;



                // Set the initial and end boundary conditions

                Eigen::Matrix<double, 4, 3> initTransBC, endTransBC;
                Eigen::Matrix<double, 2, 3> initRotBC, endRotBC;

                initRotBC.setZero();
                endRotBC.setZero();

                initTransBC.setZero();
                initTransBC.row(0) = startPos.transpose();
                // initTransBC.row(1) = (segment_dir * v_mean).transpose();

                endTransBC = initTransBC;
                endTransBC.row(0) = goalPos.transpose();
                // endTransBC.row(1) = (segment_dir * v_mean).transpose();

                // std::cout << "Optimizer matrices:\n";
                // std::cout << "Initial Orientation: \n" << startOrient << std::endl;
                // std::cout << "Goal Orientation: \n" << goalOrient << std::endl;
                // Initialize the optimizer
                std::shared_ptr<DroneOptimizer> optimizer(std::make_shared<DroneOptimizer>(startPos, diff_flat_startOrient, goalPos, diff_flat_goalOrient, initTransBC, initRotBC, endTransBC, endRotBC, t_i, t_i1));
                // std::shared_ptr<DroneOptimizer> optimizer(std::make_shared<DroneOptimizer>(startPos, startOrient, goalPos, goalOrient, t_i, t_i1));


                t_i=t_i1; 

                

                if (configuration_ != nullptr)
                {
                    // optimizer->loadConfig(configFile_);
                    optimizer->configure(configuration_);
                }
                else
                {
                    throw std::runtime_error("Optimizer configuration not set");
                }

                

                if (!polytopesEmpty){

                    Polytope<double, 3>& freeSpacePolytope = freeSpacePolytopes_[i];

                    if ((freeSpacePolytope.getA().array().allFinite()))
                    {

                        // std::cout << "Free Space Polytope: " << freeSpacePolytope << std::endl;
                        optimizer->setFreeSpacePolytope(freeSpacePolytope);

                    }

                }

                if (!subMapsEmpty)
                {

                    std::vector<uncertainPointXYZ> subMap = subMaps_[i];
                    
                    optimizer->setMap(subMap);
                    
                }
                          
                
                // Perform optimization
                optimizer->optimize();

                std::cout << "Optimizer finished" << std::endl;

                splinePair splines = std::make_pair(optimizer->getTransSpline(), optimizer->getRotSpline());;
                splines_.push_back(splines);
                
                // Extract the optimized control points
                // std::cout << "Optimized Translation control points:\n" << optimizer->getTranslationControlPoints() << std::endl;
                // std::cout << "Optimized Rotation control points:\n" << optimizer->getRotationControlPoints() << std::endl;
                

                // Update the waypoints with the optimized control points
                // addWaypoints(optimizer);
            }

            // path = PathGeometric(si_, waypoints_);

            std::cout << "Path smoothed" << std::endl;

        }

        Eigen::Vector3d PathSmoother::extractPosition(const SE3State_ *state)
        {
            // Implementation to extract the position from the state
            // This is a placeholder and should be replaced with actual extraction logic
            Eigen::Vector3d position(state->getX(), state->getY(), state->getZ());
            // Example: position << state->as<SomeStateType>()->getX(), state->as<SomeStateType>()->getY(), state->as<SomeStateType>()->getZ();
            return position;
        }

        Eigen::Matrix3d PathSmoother::extractOrientation(const SE3State_ *state)
        {
            // Implementation to extract the orientation from the state
            // This is a placeholder and should be replaced with actual extraction logic
            Eigen::Quaterniond q(state->rotation().w, state->rotation().x, state->rotation().y, state->rotation().z);
            Eigen::Matrix3d orientation(q);
            // Example: orientation = state->as<SomeStateType>()->getRotationMatrix();
            return orientation;
        }

        void PathSmoother::setStateFromEigen(ompl::base::State *state, const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation)
        {
            // Implementation to set the position and orientation of the state
            
            
            state->as<SE3State_>()->setXYZ(position[0], position[1], position[2]);
            Eigen::Quaterniond q(orientation);
            state->as<SE3State_>()->rotation().w = q.w();
            state->as<SE3State_>()->rotation().x = q.x();
            state->as<SE3State_>()->rotation().y = q.y();
            state->as<SE3State_>()->rotation().z = q.z();


            // Example: state->as<SomeStateType>()->setX(position[0]); state->as<SomeStateType>()->setY(position[1]); state->as<SomeStateType>()->setZ(position[2]);
            // Example: state->as<SomeStateType>()->setRotationMatrix(orientation);
        }

        void PathSmoother::addWaypoints(std::shared_ptr<DroneOptimizer> optimizer)
        {
            // TODO: add times as well

            double initialTime{optimizer->getInitialTime()};
            double finalTime{optimizer->getFinalTime()};



            std::vector<Eigen::Vector3d> transW = optimizer->sampleTranslations(numWaypointsPerSegment_);
            std::vector<Eigen::Matrix3d> rotW = optimizer->sampleRotations(numWaypointsPerSegment_);

            Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(numWaypointsPerSegment_, initialTime, finalTime);



            for (size_t i{0}; i < numWaypointsPerSegment_-1 ; ++i )
            {
                if (!transW[i].allFinite() || !rotW[i].array().isFinite().all()) {
                    std::cerr << "Warning: NaN detected in translation or rotation at waypoint " << i << std::endl;
                    break;
                }

                ompl::base::State* temp = si_->allocState();

                // std::cout << "Pos: " << transW[i].transpose() << std::endl;
                times_.push_back(times[i]);

                setStateFromEigen(temp, transW[i], rotW[i].transpose());


                waypoints_.push_back(temp);
            }

        }

        void PathSmoother::setParameters()
        {
            // Set parameters for the Gaussian smoother
            // This is a placeholder and should be replaced with actual parameter setting logic
            numWaypointsPerSegment_ = configuration_->num_waypoints_per_segment; // Example value, adjust as needed
            v_mean = configuration_->mean_velocity; // Example value, adjust as needed

            setHorizon(configuration_->smoother_horizon); // Set the horizon from the configuration
        }


        Eigen::Quaterniond PathSmoother::computeAttitudeFromAccelPreserveHeading(
                const Eigen::Quaterniond &q_old,
                const Eigen::Vector3d &accel_world,
                const Eigen::Vector3d &g
            )
        {
            // Compute net “force” (thrust direction) in world
            Eigen::Vector3d f = accel_world + g;
            double fnorm = f.norm();
            if (fnorm < 1e-8) {
                // degenerate, return old orientation
                return q_old;
            }
            Eigen::Vector3d y_des = f / fnorm;  // desired body-y in world frame

            // Extract old body z in world frame
            Eigen::Vector3d z_old = q_old * Eigen::Vector3d::UnitZ();

            // Project z_old onto plane perpendicular to y_des to get new x direction
            Eigen::Vector3d z_proj = z_old - y_des * (y_des.dot(z_old));
            double zproj_norm = z_proj.norm();
            Eigen::Vector3d z_des;
            if (zproj_norm < 1e-6) {
                // fallback: choose some axis orthogonal to y_des
                // pick world X or Y
                Eigen::Vector3d tmp(0,0,1);
                if (std::abs(y_des.dot(tmp)) > 0.9) {
                    tmp = Eigen::Vector3d(1,0,0);
                }
                z_des = (tmp - y_des * (y_des.dot(tmp))).normalized();
            } else {
                z_des = z_proj / zproj_norm;
            }

            // y_des = z_des × x_des
            Eigen::Vector3d x_des = y_des.cross(z_des);

            // Build rotation matrix: columns are body axes in world frame
            Eigen::Matrix3d R_des;
            R_des.col(0) = x_des;
            R_des.col(1) = y_des;
            R_des.col(2) = z_des;

            Eigen::Quaterniond q_des(R_des);

            // Ensure same quaternion hemisphere (so slerp preserves direction)
            if (q_des.dot(q_old) < 0) {
                q_des.coeffs() *= -1;
            }

            return q_des.normalized();
        }
   

}
}