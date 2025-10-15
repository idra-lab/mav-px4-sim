#ifndef BENCHMARKING_APACE_BENCHMARKING_HPP
#define BENCHMARKING_APACE_BENCHMARKING_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <benchmarking/apace/full_smoother.hpp>

namespace benchmarking {
namespace apace {
    class FullSmootherBenchmarkingNode : public rclcpp::Node {
    public:
        FullSmootherBenchmarkingNode();
        ~FullSmootherBenchmarkingNode(){};
    private:
        std::shared_ptr<FullSmoother> full_smoother_planner_;


        void loadConfigurationFile(std::string configFile);

        void loadMap();

        void planPath();

        void publishPath();
        void publishMap();
    };

} // namespace apace
} // namespace benchmarking


#endif // BENCHMARKING_APACE_BENCHMARKING_HPP