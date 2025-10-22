#include "benchmarking/apace/full_smoother.hpp"

namespace benchmarking {
namespace apace {
// Constructor
FullSmoother::FullSmoother()
{
	
	space = std::make_shared<ob::SE3StateSpace>();
	// set the bounds for the R^3
	ob::RealVectorBounds bounds(3);
  bounds.setLow(-boundingBoxSize);
  bounds.setHigh(boundingBoxSize);

  space->setBounds(bounds);

	// construct an instance of  space information from this state space
	si = std::make_shared<ob::SpaceInformation>(space);


  // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&FullSmoother::isStateValid, this, std::placeholders::_1 ));

	// si->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(si));
	si->setMotionValidator(std::make_shared<RayCastMotionValidator>(si, this->map));

	// create a start state
	ob::ScopedState<> start(space);
	
	start->as<SE3State_>()->setXYZ(0,0,0);
	start->as<SE3State_>()->rotation().setIdentity();

	// std::cout << "Start set" << std::endl;

	// create a goal state
	ob::ScopedState<> goal(space);

	goal->as<SE3State_>()->setXYZ(1,1,1);
	goal->as<SE3State_>()->rotation().setIdentity();

	// std::cout << "Goal set" << std::endl;
	
	// create a problem instance
	pdef = std::make_shared<ob::ProblemDefinition>(si);
	// std::cout << "Problem definition" << std::endl;

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	// std::cout << "Add start and goal to Problem definition" << std::endl;

    // set Optimizattion objective
	pdef->setOptimizationObjective(getVisibilityObj(si));
	// std::cout << "set optimizaton objective to Problem definition" << std::endl;

    // create a planner for the defined space
	o_plan = std::make_shared<og::RRTstar>(si);
	// std::cout << "Create planner object" << std::endl;

    // set the problem we are trying to solve for the planner
	o_plan->setProblemDefinition(pdef);
	// std::cout << "Set problem definition in planner" << std::endl;

    // perform setup steps for the planner
	o_plan->setup();
	// std::cout << "Planner setup" << std::endl;

	// INFO("FullSmoother Initialized");
	// std::cout << "Initialization Completed" << std::endl;

}

// Constructor
FullSmoother::FullSmoother(Eigen::Vector3d startPos, Eigen::Vector3d goalPos)
{
	

	space = std::make_shared<ob::SE3StateSpace>();
	// std::cout << "space Initialized" << std::endl;
	
	// set the bounds for the R^3
	ob::RealVectorBounds bounds(3);
    bounds.setLow(-boundingBoxSize);
    bounds.setHigh(boundingBoxSize);
	// std::cout << "bounds Initialized" << std::endl;
 
    space->setBounds(bounds);

	// std::cout << "bounds set" << std::endl;

	// construct an instance of  space information from this state space
	si = std::make_shared<ob::SpaceInformation>(space);

	// std::cout << "si Initialized" << std::endl;

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&FullSmoother::isStateValid, this, std::placeholders::_1 ));
	// std::cout << "Statte validity set" << std::endl;

	// si->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(si));
	si->setMotionValidator(std::make_shared<RayCastMotionValidator>(si, this->map));

	// create a start state
	ob::ScopedState<> start(space);
	
	start->as<SE3State_>()->setXYZ(startPos[0],startPos[1],startPos[2]);
	start->as<SE3State_>()->rotation().setIdentity();

	// std::cout << "Start set" << std::endl;

	// create a goal state
	ob::ScopedState<> goal(space);

	goal->as<SE3State_>()->setXYZ(goalPos[0],goalPos[1],goalPos[2]);
	goal->as<SE3State_>()->rotation().setIdentity();

	// std::cout << "Goal set" << std::endl;
	
	// create a problem instance
	pdef = std::make_shared<ob::ProblemDefinition>(si);
	// std::cout << "Problem definition" << std::endl;

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	// std::cout << "Add start and goal to Problem definition" << std::endl;

    // set Optimizattion objective
	pdef->setOptimizationObjective(getVisibilityObj(si));

	
	// std::cout << "set optimizaton objective to Problem definition" << std::endl;

    // create a planner for the defined space
	o_plan = std::make_shared<og::RRTstar>(si);
	// std::cout << "Create planner object" << std::endl;

    // set the problem we are trying to solve for the planner
	o_plan->setProblemDefinition(pdef);
	// std::cout << "Set problem definition in planner" << std::endl;

    // perform setup steps for the planner
	o_plan->setup();
	// std::cout << "Planner setup" << std::endl;

	// INFO("FullSmoother Initialized");
	// std::cout << "Initialization Completed" << std::endl;

}



FullSmoother::FullSmoother(std::string configFile)
{

	loadConfigFile(configFile);


	
	space = std::make_shared<ob::SE3StateSpace>();
	std::cout << "space Initialized" << std::endl;

	double maxAngle = 0.3;  // radians
	
	// set the bounds for the R^3
	ob::RealVectorBounds bounds(3);
    bounds.setLow(0, plannerConfig_->globalBox.x_min);
    bounds.setLow(1, plannerConfig_->globalBox.y_min);
    bounds.setLow(2, plannerConfig_->globalBox.z_min);

    bounds.setHigh(0, plannerConfig_->globalBox.x_max);
    bounds.setHigh(1, plannerConfig_->globalBox.y_max);
    bounds.setHigh(2, plannerConfig_->globalBox.z_max);

	std::cout << "bounds Initialized" << std::endl;
 
    space->setBounds(bounds);

	std::cout << "bounds set" << std::endl;

	// construct an instance of  space information from this state space
	si = std::make_shared<ob::SpaceInformation>(space);



	std::cout << "si Initialized" << std::endl;

    // set state validity checking for this space
	si->setStateValidityChecker(std::bind(&FullSmoother::isStateValid, this, std::placeholders::_1 ));
	std::cout << "Statte validity set" << std::endl;

	// si->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(si));
	si->setMotionValidator(std::make_shared<RayCastMotionValidator>(si, this->map));

	// create a start state
	// ob::ScopedState<> start(cs);
	
	// start->as<SE3State_>()->setXYZ(current_position_[0],current_position_[1],current_position_[2]);
	// start->as<SE3State_>()->rotation().setIdentity();

	// std::cout << "Start set" << std::endl;

	// create a goal state
	// ob::ScopedState<> goal(cs);

	// goal->as<SE3State_>()->setXYZ(global_goal[0],global_goal[1],global_goal[2]);
	// goal->as<SE3State_>()->rotation().setIdentity();


	// std::cout << "Goal set" << std::endl;
	
	// create a problem instance
	pdef = std::make_shared<ob::ProblemDefinition>(si);
	std::cout << "Problem definition" << std::endl;

	// set the start and goal states
	// pdef->setStartAndGoalStates(start, goal);
	// std::cout << "Add start and goal to Problem definition" << std::endl;

    // set Optimizattion objective
	pdef->setOptimizationObjective(getVisibilityObj(si));
	std::cout << "set optimizaton objective to Problem definition" << std::endl;

	ob::OptimizationObjectivePtr obj = pdef->getOptimizationObjective();
	

    // create a planner for the defined space
	o_plan = std::make_shared<og::RRTstar>(si);
	std::cout << "Create planner object" << std::endl;

    // set the problem we are trying to solve for the planner
	o_plan->setProblemDefinition(pdef);
	std::cout << "Set problem definition in planner" << std::endl;

    // perform setup steps for the planner
	o_plan->setup();
	std::cout << "Planner setup" << std::endl;

	// INFO("FullSmoother Initialized");
	// std::cout << "Initialization Completed" << std::endl;

}

// Destructor
FullSmoother::~FullSmoother()
{
}


bool FullSmoother::setStart(double x, double y, double z, double qw, double qx, double qy, double qz)
{
	ob::ScopedState<> start(space);
	
	start->as<SE3State_>()->setXYZ(x, y, z);
	start->as<SE3State_>()->rotation().w = qw;
	start->as<SE3State_>()->rotation().x = qx;
	start->as<SE3State_>()->rotation().y = qy;
	start->as<SE3State_>()->rotation().z = qz;

	ob::State *temp =  space->allocState();
	temp->as<SE3State_>()->setXYZ(x, y, z);
	temp->as<SE3State_>()->rotation().w = qw;
	temp->as<SE3State_>()->rotation().x = qx;
	temp->as<SE3State_>()->rotation().y = qy;
	temp->as<SE3State_>()->rotation().z = qz;
	
	if(isCollisionFree(temp)) // Check if the start state is valid
	{	
		space->enforceBounds(start.get());
		pdef->clearStartStates();
		pdef->addStartState(start);

		// set the problem we are trying to solve for the planner
		o_plan->setProblemDefinition(pdef);
		// std::cout << "Set problem definition in planner" << std::endl;

		// perform setup steps for the planner
		o_plan->setup();

		return true;
	}
	else
	{
		std::cout << "Start state: " << x << " " << y << " " << z << " invalid\n";
		return false;
	}
}

bool FullSmoother::setGoal(double x, double y, double z, double qw, double qx, double qy, double qz)
{
	ob::ScopedState<> goal(space);
	goal->as<SE3State_>()->setXYZ(x, y, z);
	goal->as<SE3State_>()->rotation().w = qw;
	goal->as<SE3State_>()->rotation().x = qx;
	goal->as<SE3State_>()->rotation().y = qy;
	goal->as<SE3State_>()->rotation().z = qz;
	
	ob::State *temp =  space->allocState();
	temp->as<SE3State_>()->setXYZ(x, y, z);
	temp->as<SE3State_>()->rotation().w = qw;
	temp->as<SE3State_>()->rotation().x = qx;
	temp->as<SE3State_>()->rotation().y = qy;
	temp->as<SE3State_>()->rotation().z = qz;
	
	// if(isStateValid(temp)) // Check if the goal state is valid
	{	
		// DBG("Goal point set to: " << x << " " << y << " " << z);
		space->enforceBounds(goal.get());
		pdef->clearGoal();
		pdef->setGoalState(goal);

		// set the problem we are trying to solve for the planner
		o_plan->setProblemDefinition(pdef);
		// std::cout << "Set problem definition in planner" << std::endl;

		// perform setup steps for the planner
		o_plan->setup();
		
		return true;
	}
	// else
	// {
	// 	std::cout << "Goal state: " << x << " " << y << " " << z << " invalid\n";
	// 	return false;
	// }
}


void FullSmoother::updateMap(std::shared_ptr<FullUncertainOctomap> newMap)
{
	// convert octree to collision object

	this->map = newMap;

	this->map->setVoxelPadding(robotSize);

	auto* motionVal = dynamic_cast<RayCastMotionValidator*>(si->getMotionValidator().get());

	motionVal->map = this->map;

	auto* visObj = dynamic_cast<VisibilityObjective*>(pdef->getOptimizationObjective().get());


	// visObj->setMap(this->map);
	
}


void FullSmoother::setBoundingBoxSize(float size)
{
	boundingBoxSize = size;
	// ob::RealVectorBounds bounds(3);
	// bounds.setLow(-boundingBoxSize);
	// bounds.setHigh(boundingBoxSize);
	// space->setBounds(bounds);

	// TODO: Check this later
	// Vec3f origin(-boundingBoxSize/2, -boundingBoxSize/2, -boundingBoxSize/2),  range(boundingBoxSize, boundingBoxSize, boundingBoxSize);
	Vec3f origin(-boundingBoxSize/2, -boundingBoxSize/2, -boundingBoxSize/2),  range(2*boundingBoxSize, 2*boundingBoxSize, 2*boundingBoxSize);
	setDecomposer(origin, range);
}

void FullSmoother::setBoundingBoxSize() 
	{

		if (plannerConfig_->globalBox.x_max == NULL)
		{
			boundingBoxSize = plannerConfig_->globalBox.size;
			Vec3f origin(-boundingBoxSize/2, -boundingBoxSize/2, -boundingBoxSize/2),  range(boundingBoxSize, boundingBoxSize, boundingBoxSize);
			setDecomposer(origin, range);
		}
		else
		{
			boundingBoxSize = plannerConfig_->globalBox.size;

			global_xbound = plannerConfig_->globalBox.x_max - plannerConfig_->globalBox.x_min;
			global_ybound = plannerConfig_->globalBox.y_max - plannerConfig_->globalBox.y_min;
			global_zbound = plannerConfig_->globalBox.z_max - plannerConfig_->globalBox.z_min;

			Vec3f origin(plannerConfig_->globalBox.x_min, plannerConfig_->globalBox.y_min, plannerConfig_->globalBox.z_min), range(global_xbound, global_ybound, global_zbound);

			setDecomposer(origin, range);


		}
	};

void FullSmoother::setLocalBoxSize() 
{
	local_xbound = (plannerConfig_->localBox.x_max - plannerConfig_->localBox.x_min);
	local_ybound = (plannerConfig_->localBox.y_max - plannerConfig_->localBox.y_min);
	local_zbound = (plannerConfig_->localBox.z_max - plannerConfig_->localBox.z_min);
};


bool FullSmoother::replan(void)
{	
	if(path_smooth != NULL)
	{
		og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
		// DBG("Total Points:" << path->getStateCount ());
		double distance;
		if(pdef->hasApproximateSolution())
		{
			// DBG("Goal state not satisfied and distance to goal is: " << pdef->getSolutionDifference());
			replan_flag = true;
		}
		else
		{
			for (std::size_t idx = 0; idx < path->getStateCount (); idx++)
			{
				if(!replan_flag)
				{
					replan_flag = !isStateValid(path->getState(idx));
				}
				else
					break;
			}
		}
	}
	if(replan_flag)
	{
		pdef->clearSolutionPaths();
		// DBG("Replanning");
		plan();
		return true;
	}
	else
	{
		// DBG("Replanning not required");
		return false;
	}
}

void FullSmoother::reset(void)
{	
	// create a problem instance
	pdef = std::make_shared<ob::ProblemDefinition>(si);
	// std::cout << "Problem definition" << std::endl;

    // set Optimizattion objective
	pdef->setOptimizationObjective(getVisibilityObj(si));

	// std::cout << "set optimizaton objective to Problem definition" << std::endl;

    // create a planner for the defined space
	o_plan = std::make_shared<og::RRTstar>(si);
	// std::cout << "Create planner object" << std::endl;

    // set the problem we are trying to solve for the planner
	o_plan->setProblemDefinition(pdef);
	
	path_smooth = NULL;
	replan_flag = false;

}

void FullSmoother::initializeCost()
{
	auto visObjPtr = std::dynamic_pointer_cast<VisibilityObjective>(pdef->getOptimizationObjective());
	// Make a thread-safe deep copy of the current octomap and pass it to the objective
	// std::shared_ptr<FullUncertainOctomap> map_copy;
	// {
	// 	// std::mutex &m = this->map->getMutex();
	// 	// std::lock_guard<std::mutex> lock(m);
	// 	map_copy = std::make_shared<FullUncertainOctomap>(*this->map); // requires FullUncertainOctomap copy ctor
	// }
	visObjPtr->setMap(map);
	visObjPtr->setStart(pdef->getStartState(0));
	visObjPtr->setGoal(pdef->getGoal()->as<ob::GoalState>()->getState());
	visObjPtr->initialize();

	
}

bool FullSmoother::findIntersection(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& center, double R, Eigen::Vector3d& new_end)
{

	std::cout << "Center: " << center.transpose() << " R: " << R << std::endl;
	Eigen::Vector3d segment = p1 - p0;
	double segment_length = segment.norm();
	Eigen::Vector3d u = segment.normalized();

	std::cout << "Segment length: " << segment_length << std::endl;

	double R2 = R* R;

	Eigen::Vector3d dist_origin_to_center = p0 - center;
	

	double a = 1.0; // d.dot(d) == 1 since d is normalized
	double b = 2.0 * u.dot(dist_origin_to_center);
	double c = dist_origin_to_center.squaredNorm() - R2;

	double disc = b * b - 4 * a * c;

	if (disc < 0.0)
	{
		std::cout << "No intersection\n";
		return false; // No intersection
	}

	double sqrt_disc = std::sqrt(disc);

	double t1 = (-b - sqrt_disc) / (2 * a);
	double t2 = (-b + sqrt_disc) / (2 * a);

	std::cout << "t1: " << t1 << " t2: " << t2 << std::endl;

	bool inside = dist_origin_to_center.squaredNorm() < R * R;

	std::cout << "Inside: " << inside << std::endl;

    double t = inside ? std::max(t1, t2) : std::min(t1, t2);

	std::cout << "t: " << t << std::endl;

    if (t < 0.0 || t > segment_length) return false;


    new_end = p0 + u * t;

	
    return true;
}

void FullSmoother::cutPathToFovSphere(og::PathGeometric* path, const Eigen::Vector3d& center, const double& radius, og::PathGeometric* new_path){
	double R2 = radius * radius;
    unsigned int N = path->getStateCount();
    if (N < 2)
        return ;

    // get start translation as center
    const ob::SE3StateSpace::StateType *s0 =
        path->getState(0)->as<ob::SE3StateSpace::StateType>();

	

    // Loop through segments
    for (unsigned int i = 1; i < N; ++i)
    {
        const ob::SE3StateSpace::StateType *sprev =
            path->getState(i - 1)->as<ob::SE3StateSpace::StateType>();
        const ob::SE3StateSpace::StateType *scurr =
            path->getState(i)->as<ob::SE3StateSpace::StateType>();

			

        Eigen::Vector3d p0(sprev->getX(), sprev->getY(), sprev->getZ());
        Eigen::Vector3d p1(scurr->getX(), scurr->getY(), scurr->getZ());

		// std::cout << "P0: " << p0.transpose() << " P1: " << p1.transpose() << std::endl;

		octomap::point3d op0(p0[0], p0[1], p0[2]);

		if (!map->areNodesAndVicinityFree(op0, map->getVoxelPadding(), false))
		{
			break;
		}

		new_path->append(path->getState(i-1)); // always add the previous point
		
		Eigen::Vector3d new_end;
		bool found = findIntersection(p0, p1, center, radius, new_end);

		// std::cout << "New End: " << new_end.transpose() << " Found: " << found << std::endl;

		if (!found){ new_end = p1; }

		octomap::point3d op1(new_end[0], new_end[1], new_end[2]);

		std::vector<octomap::point3d> points;
		bool success = map->computeRay(op0, op1, points);

		if (!success)
		{
			break;
		}

		octomap::point3d last_free = op0;

		for (const auto &point : points)
		{
			// std::cout << "Point: " << point << std::endl;
			if (!map->areNodesAndVicinityFree(point, map->getVoxelPadding(), false))
			{
				std::cout << "Collision at point: " << point << std::endl;
				break;
			}
			else{

				last_free = point;
			}
		}

		Eigen::Vector3d last_free_e(last_free.x(), last_free.y(), last_free.z());

		double percentage = (last_free_e - p0).norm() / (p1 - p0).norm();



		ob::State *new_end_state =  space->allocState();


		this->space->interpolate(sprev, scurr, percentage, new_end_state);
		if (seesAtLeastOneOccupied(new_end_state))
		{
			new_path->append(new_end_state);
		}
		else
		{
			break;
		}

		
		

		// new_end_state->as<SE3State_>()->setXYZ(last_free.x(), last_free.y(), last_free.z());
		// new_end_state->as<SE3State_>()->rotation().w = sprev->rotation().w;
		// new_end_state->as<SE3State_>()->rotation().x = sprev->rotation().x;
		// new_end_state->as<SE3State_>()->rotation().y = sprev->rotation().y;
		// new_end_state->as<SE3State_>()->rotation().z = sprev->rotation().z;

		// new_path->append(new_end_state);

		

    }
}




bool FullSmoother::plan(void)
{

	initializeCost();

    // Attempt to solve the problem within solve_time seconds of planning time
	ob::PlannerStatus solved = o_plan->ob::Planner::solve(solve_time);

	if (solved)
	{
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
		// DBG("Found solution:");

		std::mutex& map_mutex = map->getMutex();

		// Lock the mutex to ensure thread safety when accessing the map
		
		// std::lock_guard<std::mutex> lock(map_mutex);

		ob::PathPtr path = pdef->getSolutionPath();
		og::PathGeometric* pth = pdef->getSolutionPath()->as<og::PathGeometric>();
		std::cout << "Initial path found with " << pth->getStateCount() << " states\n";
		pth->printAsMatrix(std::cout);

		auto octo_path = vec3ToOctomap(path);

		std::vector<std::vector<uncertainPointXYZ>> subMaps;

		subMaps = map->getLocalSubmaps(octo_path);
		
		subMapsLog_ = subMaps;

		obstacles_ = map->getObstacleMap(octo_path, local_xbound, local_ybound, local_zbound);
		
		// std::cout << "Obstacle Map Size: " << obstacles_.size() << std::endl;

		std::vector<Polytope<double, 3>> freeSpaces = getFreeSpacePolytopes(path, obstacles_);
		polytopesLog_ = freeSpaces;
		
        //Path smoothing using bspline
		PathSmoother* pathBSpline = new PathSmoother(si);

		if (plannerConfig_!= nullptr)
		{
			pathBSpline->configure(plannerConfig_);
		}
		else
		{
			throw std::runtime_error("Planner configuration not set");
		}

		pathBSpline->setFreeSpacePolytopes(freeSpaces);
		
		pathBSpline->setSubMaps(subMaps);
		
		path_smooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*path));

		pathBSpline->smoothPath(*path_smooth);

		path_times = pathBSpline->getPathTimes();

		splines_ = pathBSpline->getSplines();

		replan_flag = false;

		return true;

	}
	else
	{
		// DBG("No solution found");
        std::cout << "No solution found" << std::endl;
		return false;
	}
		

}

bool FullSmoother::isStateValid(const ob::State *state)
{
	// Remove const-ness to enforce bounds (not recommended unless necessary)
	// space->enforceBounds(const_cast<ob::State*>(state));
    // cast the abstract state type to the type we expect
	const SE3State_ *pos = state->as<SE3State_>();

    // check validity of state defined by pos
	// fcl::Vector3<double> translation(pos->getX(),pos->getY(),pos->getZ());

	const auto &rotation = pos->rotation();
	Eigen::Quaterniond quat(rotation.w, rotation.x, rotation.y, rotation.z);
	Eigen::Vector3d y_axis = quat.toRotationMatrix() * Eigen::Vector3d::UnitY() ;

	double dot_prod = y_axis.dot(Eigen::Vector3d::UnitY()); 
	if (dot_prod < 0.5) 
	{
		// If the y-axis is pointing upwards, the state is invalid
		return false;
	}


	if(!isCollisionFree(state) )// Check if the position is in a free voxel
	{
		return false;
	}

	if (!seesAtLeastOneOccupied(state) ) // Check if the camera sees at least one occupied voxel
	{
		return false;
	}



	return true;

}

bool FullSmoother::isCollisionFree(const ob::State *state, bool unknown_is_free)
{
    // cast the abstract state type to the type we expect
	const SE3State_ *pos = state->as<SE3State_>();

	octomap::point3d pos_octomap(pos->getX(), pos->getY(), pos->getZ());

    if (!map->areNodesAndVicinityFree(pos_octomap, robotSize, unknown_is_free))
	{return false;}

	return true;

}

bool FullSmoother::seesAtLeastOneOccupied(const ob::State *state)

{
	// cast the abstract state type to the type we expect
	const SE3State_ *pos = state->as<SE3State_>();

	// position and camera orientation
	Eigen::Vector3d position(pos->getX(), pos->getY(), pos->getZ());
	const auto &rotation = pos->rotation();
	Eigen::Quaterniond quat(rotation.w, rotation.x, rotation.y, rotation.z);

	// camera Z axis in world frame
	Eigen::Vector3d cam_dir = quat * Eigen::Vector3d::UnitZ();
	if (cam_dir.norm() == 0.0) return false;
	cam_dir.normalize();

	if (!map) return false;

	// cast a single ray from the camera position in the camera Z direction up to FOV distance
	double max_range = 2.0*map->getFovDistance();
	Eigen::Vector3d end_pos = position + cam_dir * max_range;

	octomap::point3d op0(position.x(), position.y(), position.z());
	octomap::point3d op1(end_pos.x(), end_pos.y(), end_pos.z());

	std::vector<octomap::point3d> ray_pts;
	bool ok = map->computeRay(op0, op1, ray_pts);
	if (!ok) return false;

	// If any point along the ray is not free (i.e. occupied), return true
	for (const auto &pt : ray_pts)
	{
		if (!map->areNodesAndVicinityFree(pt, map->getVoxelPadding(), true))
		{
			return true;
		}
	}

	return false;
}

	




// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.

ob::OptimizationObjectivePtr FullSmoother::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);


	return obj;
}

ob::OptimizationObjectivePtr FullSmoother::getVisibilityObj(const ob::SpaceInformationPtr& si)
{
	ob::OptimizationObjectivePtr obj(new VisibilityObjective(si));
	

	return obj;
}


std::vector<Eigen::Vector3f> FullSmoother::vec3ToEigen(ob::PathPtr path)
{
	std::vector<Eigen::Vector3f> eig_path;	
	og::PathGeometric* path_g = path->as<og::PathGeometric>();

	for (std::size_t idx = 0; idx < path_g->getStateCount(); idx++)
	{
		// cast the abstract state type to the type we expect
		const SE3State_ *pos = path_g->getState(idx)->as<SE3State_>();
		eig_path.push_back(Eigen::Vector3f(pos->getX(), pos->getY(), pos->getZ()));
	}
	return eig_path;
}

std::vector<octomap::point3d> FullSmoother::vec3ToOctomap(ob::PathPtr path)
{
	std::vector<octomap::point3d> oct_path;
	og::PathGeometric* path_g = path->as<og::PathGeometric>();

	for (std::size_t idx = 0; idx < path_g->getStateCount(); idx++)
	{
		// cast the abstract state type to the type we expect
		const SE3State_ *pos = path_g->getState(idx)->as<SE3State_>();

		// if (idx > 1 && (current_position_ - Eigen::Vector3d(pos->getX(), pos->getY(), pos->getZ())).norm() > map->getFovDistance())
		// {
		// 	continue; // Skip points that are too far away
		// }
		oct_path.emplace_back(pos->getX(), pos->getY(), pos->getZ());
	}
	return oct_path;
}



std::vector<Eigen::Vector<double, 8>> FullSmoother::getSmoothPath()
{
	std::vector<Eigen::Vector<double,8>> path;
	if (path_smooth == NULL)
	{
		// std::cout << "Path is not smoothed yet" << std::endl;
		return path; // Return an empty path if path_smooth is not set
	}
	for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
	{
        // cast the abstract state type to the type we expect
		const SE3State_ *pos = path_smooth->getState(idx)->as<SE3State_>();
		path.push_back(Eigen::Vector<double, 8>(pos->getX(), pos->getY(), pos->getZ(), pos->rotation().w, pos->rotation().x, pos->rotation().y, pos->rotation().z, path_times[idx]));
	}

	return path;
}

std::vector<Polytope<double, 3>> FullSmoother::getFreeSpacePolytopes(ob::PathPtr path, vec_Vec3f& obstacles)
{

	std::vector<Polytope<double, 3>> polytopes;

	// Get the path
	std::vector<ob::State*> states = path->as<og::PathGeometric>()->getStates();

	// std::vector<Eigen::Vector3d> positions;
	vec_Vecf<3> positions;

	for (int i = 0; i < states.size(); i++)
	{
		// Get the state
		ob::State* state = states[i];

		// Get the position
		const SE3State_ *pos = state->as<SE3State_>();
		// Eigen::Vector3d position(pos->getX(), pos->getY(), pos->getZ());
		positions.emplace_back(pos->getX(), pos->getY(), pos->getZ());

	}

	if (obstacles.empty())
	{
		std::cout << "No obstacles provided for free space decomposition." << std::endl;
		return polytopes; // Return empty if no obstacles are provided
	}
	
	decomposer->set_obs(obstacles);
	decomposer->set_local_bbox(Eigen::Vector3d(local_xbound, local_ybound, local_zbound)); 
	decomposer->dilate(positions, 0);

	
	const auto& polys = decomposer->get_polyhedrons();
	for (size_t i{0}; i < positions.size() -1; ++i)
	{
		vec_E<vec_Vec3f>  vertices = cal_vertices(polys[i]);
		const auto pt_inside = (positions[i] + positions[i+1]) / 2;
		LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());

		Eigen::Vector3d mean; 
		mean.setZero();

		int n_points{0};
		for (const vec_Vec3f& vertex : vertices)
		{
			for (const Vec3f& point : vertex)
			{
				mean += point.cast<double>();
				n_points++;
			}
		}

		mean /= n_points;

		Polytope<double, 3> polytope(cs.A(), cs.b());
		polytope.setCenter(mean);
		polytopes.push_back(polytope);
	}

	return polytopes;
}

/* 
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

I/O Functions

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
*/


void FullSmoother::save2File(std::string filename)
{
		
	YAML::Emitter out;
	out << YAML::BeginMap;

	// Save path states
	out << YAML::Key << "path_states" << YAML::Value << YAML::BeginSeq;
	for (std::size_t idx = 0; idx < path_smooth->getStateCount(); idx++)
	{
		const SE3State_ *pos = path_smooth->getState(idx)->as<SE3State_>();
		out << YAML::Flow << YAML::BeginSeq << pos->getX() << pos->getY() << pos->getZ() << pos->rotation().w << pos->rotation().x << pos->rotation().y << pos->rotation().z << YAML::EndSeq ;
		// << YAML::EndSeq;
		// out << YAML::Flow << YAML::BeginSeq << ;
	}
	out << YAML::EndSeq;

	// Save submaps
	int cc{0};
	out << YAML::Key << "submaps" << YAML::Value << YAML::BeginSeq;
	for (const auto& subMap : subMapsLog_)
	{
		out << YAML::BeginSeq;
		for (const auto& point : subMap)
		{
			out << YAML::Flow << YAML::BeginSeq << cc << point.x << point.y << point.z;
			for (int i = 0; i < 6; ++i) // Assuming covariance is a 3x3 matrix flattened into a 9-element array
			{
				out << point.covariance[i];
			}
			out << YAML::EndSeq;
			++cc;
		}
		out << YAML::EndSeq;
	}
	out << YAML::EndSeq;

	// Save B-splines of the optimizer
	// out << YAML::Key << "translation_control_points" << YAML::Value << YAML::BeginSeq;
	// auto transCtrlPts = optimizer->getTranslationControlPoints();
	// for (int i = 0; i < transCtrlPts.cols(); ++i)
	// {
	// 	out << YAML::Flow << YAML::BeginSeq << transCtrlPts(0, i) << transCtrlPts(1, i) << transCtrlPts(2, i) << YAML::EndSeq;
	// }
	// out << YAML::EndSeq;

	// out << YAML::Key << "rotation_control_points" << YAML::Value << YAML::BeginSeq;
	// auto rotCtrlPts = optimizer->getRotationControlPoints();
	// for (int i = 0; i < rotCtrlPts.cols(); ++i)
	// {
	// 	out << YAML::Flow << YAML::BeginSeq << rotCtrlPts(0, i) << rotCtrlPts(1, i) << rotCtrlPts(2, i) << YAML::EndSeq;
	// }
	// out << YAML::EndSeq;

	// Save polytopes
	out << YAML::Key << "polytopes" << YAML::Value << YAML::BeginSeq;
	for (const auto& polytope : polytopesLog_)
	{
		out << YAML::BeginMap;
		out << YAML::Key << "A" << YAML::Value << YAML::BeginSeq;
		for (int i = 0; i < polytope.getA().rows(); ++i)
		{
			out << YAML::Flow << YAML::BeginSeq;
			for (int j = 0; j < polytope.getA().cols(); ++j)
			{
				out << polytope.getA()(i, j);
			}
			out << YAML::EndSeq;
		}
		out << YAML::EndSeq;

		out << YAML::Key << "b" << YAML::Value << YAML::Flow << YAML::BeginSeq;
		for (int i = 0; i < polytope.getB().size(); ++i)
		{
			out << polytope.getB()(i);
		}
		out << YAML::EndSeq;
		out << YAML::EndMap;
	}
	out << YAML::EndSeq;

	out << YAML::EndMap;

	std::ofstream fout(filename);
	fout << out.c_str();

}


void FullSmoother::loadConfigFile(std::string configFile)
{
	plannerConfig_ = std::make_shared<PlannerConfig>();

	plannerConfig_->loadFromYAML(configFile);

	setParametersFromConfiguration();

}

void FullSmoother::setParametersFromConfiguration()
{
	std::cout << "Setting parameters from configuration file" << std::endl;
	// setBoundingBoxSize(plannerConfig_->boundingBoxSize);

	setBoundingBoxSize();
	
	setLocalBoxSize();

	setRobotSize(plannerConfig_->robotSize);

	setSolveTime(plannerConfig_->solve_time);

	setObstacleMapSize(plannerConfig_->obstacle_map_size);

	setGlobalGoal(plannerConfig_->global_goal, plannerConfig_->global_goal_quat);

	setGoalTolerance(plannerConfig_->goal_tolerance);


	
}



/* 


MOTION VLAIDATOR FUNCTIONS



*/
bool RayCastMotionValidator::seesOneOccupied(const ob::State *state) const
{
// cast the abstract state type to the type we expect
	const SE3State_ *pos = state->as<SE3State_>();

	// position and camera orientation
	Eigen::Vector3d position(pos->getX(), pos->getY(), pos->getZ());
	const auto &rotation = pos->rotation();
	Eigen::Quaterniond quat(rotation.w, rotation.x, rotation.y, rotation.z);

	// camera Z axis in world frame
	Eigen::Vector3d cam_dir = quat * Eigen::Vector3d::UnitZ();
	if (cam_dir.norm() == 0.0) return false;
	cam_dir.normalize();

	if (!map) return false;

	// cast a single ray from the camera position in the camera Z direction up to FOV distance
	double max_range = 2.0*map->getFovDistance();
	Eigen::Vector3d end_pos = position + cam_dir * max_range;

	octomap::point3d op0(position.x(), position.y(), position.z());
	octomap::point3d op1(end_pos.x(), end_pos.y(), end_pos.z());

	std::vector<octomap::point3d> ray_pts;
	bool ok = map->computeRay(op0, op1, ray_pts);
	if (!ok) return false;

	// If any point along the ray is not free (i.e. occupied), return true
	for (const auto &pt : ray_pts)
	{
		if (!map->areNodesAndVicinityFree(pt, map->getVoxelPadding(), true))
		{
			return true;
		}
	}

	return false;
}


bool RayCastMotionValidator::isMotionValid(const ob::State *s1, const ob::State *s2) const
{
  const SE3State_ *p_1 = s1->as<SE3State_>();
  const SE3State_ *p_2 = s2->as<SE3State_>();

  if (!seesOneOccupied(s1) || !seesOneOccupied(s2))
  {
	return false;
  }

  octomap::point3d p1(p_1->getX(), p_1->getY(), p_1->getZ());
  octomap::point3d p2(p_2->getX(), p_2->getY(), p_2->getZ());

  Eigen::Vector3d p1_vec(p_1->getX(), p_1->getY(), p_1->getZ());
  Eigen::Vector3d p2_vec(p_2->getX(), p_2->getY(), p_2->getZ());

  Eigen::Vector3d dir = p2_vec - p1_vec;

  // if (dir.norm() > map->getFovDistance()) {
  // 	return false; // If the distance is too large, return false
  // }

  dir.normalize();

  Eigen::Quaterniond quat1(p_1->rotation().w, p_1->rotation().x, p_1->rotation().y, p_1->rotation().z);

  Eigen::Vector3d cam_dir = quat1 * Eigen::Vector3d::UnitZ(); // Camera direction in world frame

  double angle = std::acos(std::clamp(cam_dir.normalized().dot(dir), -1.0, 1.0));

  double fov_lim = map->getHorizontalFOVAngle() / 2.0; // Half of the horizontal FOV angle

  // if (angle > fov_lim || angle < - fov_lim) // 45 degrees
  // {
  // 	return false; // If the angle is too large, return false
  // }

  bool success{true};

  std::vector<octomap::point3d> points;

  try
  {
    success = map->computeRay(p1, p2, points);
    
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
  
  

  for (const auto &point : points)
  {
    if (!map->areNodesAndVicinityFree(point, map->getVoxelPadding(), true))
    {
      return false;
    }
    
  }

  return true;

}


bool RayCastMotionValidator::isMotionValid(const ob::State *s1, const ob::State *s2, std::pair<ompl::base::State *, double> &lastValid) const
{

  const SE3State_ *p_1 = s1->as<SE3State_>();
  const SE3State_ *p_2 = s2->as<SE3State_>();

  if (!seesOneOccupied(s1) || !seesOneOccupied(s2))
  {
	return false;
  }

  ob::State *lastValidState = lastValid.first;
  if (lastValidState)
  {
    SE3State_ *lastValidSE3 = lastValidState->as<SE3State_>();
    lastValidSE3->setXYZ(p_1->getX(), p_1->getY(), p_1->getZ());
    lastValidSE3->rotation().setIdentity(); // or set to some meaningful rotation
    lastValid.second = 0.5; // distance to the last valid point
  }
  
  Eigen::Vector3d p1_vec(p_1->getX(), p_1->getY(), p_1->getZ());
  Eigen::Vector3d p2_vec(p_2->getX(), p_2->getY(), p_2->getZ());

  Eigen::Vector3d dir = p2_vec - p1_vec;

  // if (dir.norm() > map->getFovDistance()) {
  // 	return false; // If the distance is too large, return false
  // }
  dir.normalize();

  Eigen::Quaterniond quat1(p_1->rotation().w, p_1->rotation().x, p_1->rotation().y, p_1->rotation().z);

  Eigen::Vector3d cam_dir = quat1 * Eigen::Vector3d::UnitZ(); // Camera direction in world frame

  double angle = std::acos(std::clamp(cam_dir.normalized().dot(dir), -1.0, 1.0));

  // if (angle > M_PI_2 || angle < - M_PI_2) // 90 degrees
  // {
  // 	return false; // If the angle is too large, return false
  // }


  octomap::point3d p1(p_1->getX(), p_1->getY(), p_1->getZ());
  octomap::point3d p2(p_2->getX(), p_2->getY(), p_2->getZ());

  bool success{true};

  std::vector<octomap::point3d> points;

  success = map->computeRay(p1, p2, points);

  

  for (const auto &point : points)
  {
    if (!map->areNodesAndVicinityFree(point, map->getVoxelPadding(), true))
    {

      return false;
    }
    
  }

  return true;
  
  
}

ompl::base::Cost VisibilityObjective::stateCost(const ompl::base::State *s) const {

    double visibility = computeVisibilityCost(s);

    double cost =  (1.0 - visibility) ;
	// double cost = 0.0 ; 

    return ompl::base::Cost(cost);
}

std::pair<octomap::point3d, octomap::point3d> VisibilityObjective::computeFovAabbInWorld(const Eigen::Affine3d &cam_pose, double fov_horiz_deg, double fov_vert_deg, double z_min_cam, double z_max_cam) const
	{

		// std::cout << "Camera position: (" << cam_pose.translation().x() << ", " << cam_pose.translation().y() << ", " << cam_pose.translation().z() << ")  ";
		double h_ang = fov_horiz_deg * M_PI / 180.0;
		double v_ang = fov_vert_deg * M_PI / 180.0;

		// Precompute half extents at far plane in camera frame
		double x_max = z_max_cam * tan(h_ang);
		double y_max = z_max_cam * tan(v_ang);
		// double x_near = z_min_cam * tan(h_ang);
		// double y_max = z_min_cam * tan(v_ang);

		// 8 corners in camera frame (x, y, z):
		std::vector<Eigen::Vector4d> corners_cam;
		corners_cam.reserve(8);

		// Near plane:
		corners_cam.emplace_back(  x_max,  y_max,  z_min_cam, 1);
		corners_cam.emplace_back(  x_max, -y_max,  z_min_cam, 1);
		corners_cam.emplace_back( -x_max,  y_max,  z_min_cam, 1);
		corners_cam.emplace_back( -x_max, -y_max,  z_min_cam, 1);

		// Far plane:
		corners_cam.emplace_back(  x_max,  y_max,  z_max_cam, 1);
		corners_cam.emplace_back(  x_max, -y_max,  z_max_cam, 1);
		corners_cam.emplace_back( -x_max,  y_max,  z_max_cam, 1);
		corners_cam.emplace_back( -x_max, -y_max,  z_max_cam, 1);

		// Transform corners to world
		bool first = true;
		octomap::point3d minPt, maxPt;
		for (auto &c_cam : corners_cam) {
			Eigen::Vector4d c_w = cam_pose * c_cam;  // world coordinates
			octomap::point3d p(c_w[0], c_w[1], c_w[2]);
			if (first) {
				minPt = p;
				maxPt = p;
				first = false;
			} else {
				minPt.x() = std::min(minPt.x(), p.x());
				minPt.y() = std::min(minPt.y(), p.y());
				minPt.z() = std::min(minPt.z(), p.z());

				maxPt.x() = std::max(maxPt.x(), p.x());
				maxPt.y() = std::max(maxPt.y(), p.y());
				maxPt.z() = std::max(maxPt.z(), p.z());
			}
		}

		// std::cout << "FOV AABB min: (" << minPt.x() << ", " << minPt.y() << ", " << minPt.z() << ")"
		// 		  << "  max: (" << maxPt.x() << ", " << maxPt.y() << ", " << maxPt.z() << ")";

		return std::make_pair(minPt, maxPt);
	}


  double VisibilityObjective::computeVisibilityCost(const ob::State* state) const {

		const SE3State_ *p_ = state->as<SE3State_>();

		Eigen::Affine3d cam_pose = Eigen::Affine3d::Identity();
		cam_pose.translation() = Eigen::Vector3d(p_->getX(), p_->getY(), p_->getZ());
		cam_pose.linear() = Eigen::Matrix3d(Eigen::Quaterniond(p_->rotation().w, p_->rotation().x, p_->rotation().y, p_->rotation().z));
        auto [minPt, maxPt] = computeFovAabbInWorld(cam_pose, fov_horiz, fov_vert, z_min, z_max);
        
		double percentage = map->getPercentageOfOccupiedVoxels(minPt, maxPt);
		return percentage; // Higher cost for higher occupancy

    }

} // namespace apace
} // namespace benchmarking
