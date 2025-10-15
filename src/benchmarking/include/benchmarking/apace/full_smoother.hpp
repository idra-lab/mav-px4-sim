#ifndef BENCHMARKING_APACE_FULL_SMOOTHER_HPP
#define BENCHMARKING_APACE_FULL_SMOOTHER_HPP

// Add declarations for benchmarking::apace::FullSmoother here.

#include "planner_config.hpp"
#include "fullSmoother.hpp"

#include "fullUncertainOctomap.hpp"

// #include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
// #include <ompl/geometric/simplifiers/BSpline.h>

#include <ompl/base/goals/GoalState.h>

#include <decomp_util/line_segment.h>
#include <decomp_geometry/geometric_utils.h>
#include "decomp_util/ellipsoid_decomp.h"

#include <ompl/config.h>
#include <iostream>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

#include <optional>

#include <limits>


namespace benchmarking {
namespace apace {


namespace ob = ompl::base;
namespace og = ompl::geometric;

class FullSmoother {
public:

    typedef ob::SE3StateSpace::StateType SE3State_;
	typedef ob::ConstrainedStateSpace::StateType ConstrState_;


    FullSmoother();

	FullSmoother(Eigen::Vector3d startPos, Eigen::Vector3d goalPos);
	FullSmoother(Eigen::Vector3d startPos, Eigen::Quaterniond startAtt,  Eigen::Vector3d goalPos, Eigen::Quaterniond goalAtt);

	FullSmoother(std::string configFile);

    ~FullSmoother();


	bool setStart(double x, double y, double z, double qw, double qx, double qy, double qz);

	bool setStart(Eigen::Vector3d pos, Eigen::Quaterniond quat) {return setStart(pos[0], pos[1], pos[2], quat.w(), quat.x(), quat.y(), quat.z());};

	bool setStart(Eigen::Vector3d pos, Eigen::Matrix3d rot) {Eigen::Quaterniond quat(rot);return setStart(pos, quat);};
	
	bool setGoal(double x, double y, double z, double qw, double qx, double qy, double qz);

	bool setGoal(Eigen::Vector3d pos, Eigen::Quaterniond quat){return setGoal(pos[0], pos[1], pos[2], quat.w(), quat.x(), quat.y(), quat.z());};

	bool setGoal(Eigen::Vector3d pos, Eigen::Matrix3d rot) {Eigen::Quaterniond quat(rot);return setGoal(pos, quat);};

    void setGlobalGoal(double (&goal_array)[3], double (&global_quat_array)[4]) {
		global_goal[0] = goal_array[0];
		global_goal[1] = goal_array[1];
		global_goal[2] = goal_array[2];

		global_goal_quat_ = Eigen::Quaterniond(global_quat_array[3], global_quat_array[0], global_quat_array[1], global_quat_array[2]);
	};

    Eigen::Vector3d getGlobalGoal() const {return global_goal;};

	Eigen::Quaterniond getGlobalGoalQuat() const {return global_goal_quat_;};

    void updateMap(std::shared_ptr<FullUncertainOctomap> newMap);

    void setDecomposer(Vec3f origin, Vec3f range) { decomposer = std::make_shared<EllipsoidDecomp3D>(origin, range); };

	void cutPathToFovSphere(og::PathGeometric* path, const Eigen::Vector3d& center, const double& radius, og::PathGeometric* new_path);

	bool findIntersection(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& center, double R, Eigen::Vector3d& new_end);

	
	bool plan(void);

	bool replan(void);

	void reset();

	std::vector<Eigen::Vector<double, 8>> getSmoothPath();

	// Parameters setters
	void setBoundingBoxSize(float size) ;
	void setBoundingBoxSize() ;
	
	void setGlobalGoal(Eigen::Vector3d goal) {global_goal = goal;};


	void setLocalBoxSize(float x, float y, float z) { local_xbound = x; local_ybound = y; local_zbound = z; };
	void setLocalBoxSize();

	void setRobotSize(float size) { robotSize = size; if(map){this->map->setVoxelPadding(size);} };
	double getRobotSize() const { return robotSize; };

	void setSolveTime(double time) { solve_time = time; };
	void setObstacleMapSize(float size) { obstacleMapSize = size; };

	void setGoalTolerance(float tolerance) { goalTolerance = tolerance	; };
	double getGoalTolerance() const { return goalTolerance; };

	float getMapResolution() const {return plannerConfig_->resolution;}

	vec_Vec3f getObstacles() const {return obstacles_;}

	PlannerConfig& getPlannerConfig() const {return *plannerConfig_;}
	
	std::vector<Eigen::Vector3f> vec3ToEigen(ob::PathPtr path);

	std::vector<octomap::point3d> vec3ToOctomap(ob::PathPtr path);

	std::vector<Polytope<double, 3>> getFreeSpacePolytopes(ob::PathPtr path, vec_Vec3f& obstacles);

	ob::PathPtr getPath() const {return pdef->getSolutionPath();};

	void save2File(std::string filename);

	void loadConfigFile(std::string configFile);

	std::shared_ptr<FullUncertainOctomap> getMap() const {return map;};

	void setCurrentPosition(Eigen::Vector3d pos) {current_position_ = pos;};

	void setCurrentVelocity(Eigen::Vector3d vel) {current_velocity_ = vel;};

	void setCurrentOrientation(Eigen::Quaterniond q) {current_orientation_ = q;};
	void setNextPosition(Eigen::Vector3d pos) {next_position_ = pos;};
	void setNextVelocity(Eigen::Vector3d vel) {next_velocity_ = vel;};
	void setNextOrientation(Eigen::Quaterniond q) {next_orientation_ = q;};
	Eigen::Quaterniond getCurrentOrientation() const {return current_orientation_;};
	Eigen::Vector3d getCurrentPosition() const {return current_position_;};
	Eigen::Vector3d getNextPosition() const {	return next_position_;};
	Eigen::Vector3d getCurrentVelocity() const {	return current_velocity_;		};
	Eigen::Quaterniond getNextOrientation() const {	return next_orientation_;};

	void setHorizontalFov(float angle) { map->setHorizontalFOVAngle(angle); };
	float getHorizontalFov() const { return map->getHorizontalFOVAngle(); };


	void initializeCost();

private:
  // Add private member variables and methods here.

  	Eigen::Vector3d current_position_{0, 0, 0}, current_velocity_{0, 0, 0}, next_position_{0, 0, 0}, next_velocity_{0, 0, 0};
	Eigen::Quaterniond current_orientation_{1, 0, 0, 0}, next_orientation_{1, 0, 0, 0};

	double alpha{0.1}, beta_{0.9};

	// construct the state space we are planning in
	std::shared_ptr<ob::SE3StateSpace> space;

	// construct an instance of  space information from this state space
	std::shared_ptr<ob::SpaceInformation> si;

	// create a problem instance
	std::shared_ptr< ob::ProblemDefinition> pdef;

	// FullUncertainPlanner instance
	std::shared_ptr<og::RRTstar> o_plan;

	og::PathGeometric* path_smooth = NULL;
	og::PathGeometric* fallback_path = NULL;

	std::vector<float> path_times;

	float boundingBoxSize{10.0}, local_xbound{10.0}, local_ybound{10.0}, local_zbound{10.0}, global_xbound{10.0}, global_ybound{10.0}, global_zbound{10.0};
	float obstacleMapSize{10.0};

	Eigen::Vector3d global_goal{30, 2, 0};
	Eigen::Quaterniond global_goal_quat_{1, 0, 0, 0}; // global goal orientation


	vec_Vec3f obstacles_;

	float robotSize{0.2};

	bool replan_flag = true;

	double solve_time{1.0};
	double goalTolerance{0.1};

	float horizontal_fov_angle{M_PI_4}; // 45 degrees in radians


	bool isStateValid(const ob::State *state);

	bool isCollisionFree(const ob::State *state, bool unknown_is_free = true);

	bool seesAtLeastOneOccupied(const ob::State *state);

	ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
	ob::OptimizationObjectivePtr getVisibilityObj(const ob::SpaceInformationPtr& si);

	std::shared_ptr<FullUncertainOctomap> map;
	std::shared_ptr<EllipsoidDecomp3D> decomposer;


	/* 
		I/O
	*/
	std::string smootherConfigFile_{"../resources/configuration/smoother.yaml"};

	std::string plannerConfigFile_{"../resources/configuration/planner.yaml"};

	std::string plannerResultFile_{"../results/tests/planner/run.yaml"};

	std::shared_ptr<PlannerConfig> plannerConfig_;

	void setParametersFromConfiguration();

	/// @brief Sets the constraints defined in the configuration class into the Smoother
	/// @param pathBSpline Is the smoother object
	/* 
		The class enters the smoother and accesses the underlying optimizer, using class methods to define constraints and optimization parameters
	*/
	void configSmoother(og::FullSmoother* pathBSpline);

	/* 
		LOGGING
	*/
	std::vector<Polytope<double, 3>> polytopesLog_;
	std::vector<std::vector<uncertainPointXYZ>> subMapsLog_;
};



class RayCastMotionValidator : public ompl::base::MotionValidator {

	public: 
	typedef ob::SE3StateSpace::StateType SE3State_;
	typedef ob::ConstrainedStateSpace::StateType ConstrState_;

	std::shared_ptr<FullUncertainOctomap> map;

	RayCastMotionValidator(const ompl::base::SpaceInformationPtr &si) : MotionValidator(si) {}
	RayCastMotionValidator(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<FullUncertainOctomap> m_map) : MotionValidator(si), map(m_map) {}

	bool isMotionValid(const ob::State *s1, const ob::State *s2) const;

	bool isMotionValid(const ob::State *s1, const ob::State *s2, std::pair<ompl::base::State *, double> &lastValid) const;

	bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override {return isMotionValid(s1, s2); }

	bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override {return isMotionValid(s1, s2, lastValid);}
};




class VisibilityObjective : public ompl::base::OptimizationObjective {
public:

	typedef ob::SE3StateSpace::StateType SE3State_;
	typedef ob::ConstrainedStateSpace::StateType ConstrState_;

	std::shared_ptr<FullUncertainOctomap> map;

	VisibilityObjective(const ompl::base::SpaceInformationPtr &si) : OptimizationObjective(si) {}
	VisibilityObjective(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<FullUncertainOctomap> m_map) : OptimizationObjective(si), map(m_map) {}
    VisibilityObjective(const ompl::base::SpaceInformationPtr &si,std::shared_ptr<FullUncertainOctomap> map, double alpha, double beta): ompl::base::OptimizationObjective(si), map(map),alpha_(alpha), beta_(beta) {}

    ompl::base::Cost stateCost(const ompl::base::State *s) const override;

    bool isCostBetterThan(ob::Cost c1, ob::Cost c2) const override { 	return c1.value() < c2.value();}

    ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const override{	return this->combineCosts(this->stateCost(s1), this->stateCost(s2));}

    virtual ob::Cost combineCosts(ob::Cost c1, ob::Cost c2) const override { 	return ob::Cost(c1.value() + c2.value()); }

    virtual ob::Cost identityCost() const override {return ob::Cost(0.0);};
    virtual ob::Cost infiniteCost() const override {return ob::Cost(std::numeric_limits<double>::infinity());};

    void setGoal(const ompl::base::State *goal) { goalState_ = goal; }
	void setStart(const ompl::base::State *start) { startState_ = start; }

	void setMap(std::shared_ptr<FullUncertainOctomap> m_map) { map = m_map; }

	void setParameters(double alpha, double beta) { alpha_ = alpha; beta_ = beta; }

	void initialize(){ double max_distance = si_->distance(startState_, goalState_);}

private:

    const ompl::base::State *goalState_, *startState_;
    double alpha_{0.1}, beta_{0.9};
	double max_distance{1.0};

	double fov_horiz{30.0}, fov_vert{30.0}, z_min{5.0}, z_max{20.0}; // Dimensions of the camera's FOV box

	std::pair<octomap::point3d, octomap::point3d> computeFovAabbInWorld(const Eigen::Affine3d &cam_pose, double fov_horiz_deg, double fov_vert_deg, double z_min_cam, double z_max_cam) const;

    double computeVisibilityCost(const ob::State* state) const;
};


} // namespace apace
} // namespace benchmarking

#endif // BENCHMARKING_APACE_FULL_SMOOTHER_HPP