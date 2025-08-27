#include "rclcpp/rclcpp.hpp"


#include "path_service/simulation_2d.hpp"
#include "path_service/srv/get_path.hpp"
#include "path_service/path_publisher.hpp"
#include <memory>
#include <chrono>








int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node_path_generator = std::make_shared<Path_generator>(); // Initialize correct map with ros params. Or edit the constructor


    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service ready.");

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_path_generator);
    exec.spin();


    rclcpp::shutdown();
    return 0;
}
