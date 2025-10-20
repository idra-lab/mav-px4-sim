#ifndef APACE_BENCHMARKING_PATH_SMOOTHER_HPP
#define APACE_BENCHMARKING_PATH_SMOOTHER_HPP

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include "fisherBinghamOptimizer.hpp"



namespace benchmarking {
namespace apace {



        class PathSmoother : protected ompl::geometric::PathSimplifier
        {
            
        public:
            typedef ompl::base::SE3StateSpace::StateType SE3State_;

            typedef BSpline<double, 3, 12> transSpl;
            typedef SO3Spline<double, 2, 12> rotSpl;
            typedef std::pair<transSpl , rotSpl> splinePair;

            PathSmoother(const ompl::base::SpaceInformationPtr &si) : PathSimplifier(si) {}

            void smoothPath(ompl::geometric::PathGeometric &path);

            void setFreeSpacePolytopes(const std::vector<Polytope<double, 3>> &freeSpacePolytopes){freeSpacePolytopes_ = freeSpacePolytopes;};

            // Temporary
            void setSubMap(const std::vector<Eigen::Vector3f>& subMap);
            
            void setSubMaps(const std::vector<std::vector<uncertainPointXYZ>>& subMaps) {subMaps_ = subMaps;};

            void setConfigFile(const std::string& configFile){configFile_ = configFile;};
            
            void configure(std::shared_ptr<PlannerConfig> configuration){configuration_ = configuration; setParameters();};

            std::vector<float> getPathTimes() const {return times_;};

            void setHorizon(double horizon) {horizon_ = horizon;};

            Eigen::Quaterniond computeAttitudeFromAccelPreserveHeading(
                const Eigen::Quaterniond &q_old,
                const Eigen::Vector3d &accel_world,
                const Eigen::Vector3d &g
            );

            std::vector<splinePair> getSplines() const {return splines_;};

        private:
            Eigen::Vector3d extractPosition(const SE3State_ *state);
            
            Eigen::Matrix3d extractOrientation(const SE3State_ *state);

            void setStateFromEigen(ompl::base::State *state, const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation);
            
            void addWaypoints(std::shared_ptr<DroneOptimizer> optimizer);

            void setParameters(); 

            std::vector<const ompl::base::State*> waypoints_;
            std::vector<float> times_;

            std::vector<Polytope<double, 3>> freeSpacePolytopes_;
            std::vector<std::vector<uncertainPointXYZ>> subMaps_;

            size_t numWaypointsPerSegment_{20};

            float v_mean{0.1};

            double horizon_{2000.0}; // Horizon for the smoother, used in the optimization

            std::string configFile_;
            
            std::shared_ptr<PlannerConfig> configuration_;

            std::vector<splinePair> splines_;

        };


} // namespace apace
} // namespace benchmarking

#endif // FULL_SMOOTHER_HPP