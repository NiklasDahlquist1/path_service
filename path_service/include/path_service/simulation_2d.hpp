



#ifndef SIMULATION_2D_HPP
#define SIMULATION_2D_HPP



#include <string>
#include <sstream>
//#include <any>
#include <map>




#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"


#include "path_planning/d_star_lite.hpp"
#include "path_planning/a_star.hpp"
#include "path_planning/rrt_star.hpp"




//#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>    


#include <random>
#include <math.h>
#include <chrono>
#include <filesystem>
#include <fstream>





/*


#include "geometry_msgs/msg/point.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"



#include "auction_msgs/msg/auction.hpp"
#include "auction_msgs/msg/bid.hpp"
#include "auction_msgs/msg/price_bid.hpp"
#include "auction_msgs/msg/task.hpp"
#include "auction_msgs/msg/task_allocated.hpp"
#include "auction_msgs/msg/tasks_array.hpp"
#include "auction_msgs/msg/task_finished.hpp"
#include "auction_msgs/msg/task_result.hpp"
#include "auction_msgs/msg/set_default_behavior.hpp"
    
#include "auction_msgs/msg/state_behavior.hpp"
#include "auction_msgs/msg/state.hpp"
#include "auction_msgs/srv/get_current_state.hpp"

#include "state_tracker/srv/get_state_data.hpp"
#include "state_tracker/srv/set_state_data.hpp"


#include "behaviortree_cpp/bt_factory.h"
// #include "behaviortree_cpp/loggers/bt_cout_logger.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"



#include "task_utils/available_tasks.hpp"


#include "task_client/behaviors_uav.hpp"

#include "behaviors/conditions/success_after_time.hpp"

*/





namespace simulation_2d
{


namespace constants
{
    // b,g,r format
    inline const cv::Vec3b occupied = {0, 0, 0};
    inline const cv::Vec3b free = {255, 255, 255};
    inline const cv::Vec3b target_location = {255, 0, 0};
    inline const cv::Vec3b starting_location = {0, 255, 0};  
    
    inline const cv::Vec3b planned_path_color = {210, 100, 50};
}



enum filtering_strategies
{
    radius = 0,
    share_of_tasks,
    no_filtering,
};

enum path_planning_strategies
{
    D_star = 0,
};

struct simulation_info
{
    std::string directory;
    std::string map_filename;

    filtering_strategies filtering_strategy;
    path_planning_strategies path_planning_strategy;
    double filtering_value; // radius or percentage


    size_t image_scale_factor = 8;

    cv::Mat map;
    std::vector<std::vector<int>> occupancy_grid;
    std::vector<Node> start_locations;
    std::vector<Node> target_locations;
};

struct data_one_step
{  
    simulation_info sim_info;
    int64_t cost_estimation_total_time_ms; 
    int64_t cost_estimation_average_time_ms; 
    std::vector<int64_t> cost_estimation_time_per_agent_ms; 
    int64_t optimization_time_ms;

    bool assignment_is_optimal = false; // true if the assignment is the same as the one with no filtering
    int assignment_difference_tasks; // Number of tasks that are assigned differently
    double optimal_distance; // the total distance of the assigned paths
    std::vector<std::tuple<int, double>> optimal_distance_per_agent; // optimally assigned distance per agent i (= 0 if no assignment is made)
    double error_relative_optimal = 0; // the total distance of the assigned paths

    // double optimal_assignment_error = 0;

    std::vector<Node> start_locations;
    std::vector<Node> target_locations;

    std::vector<std::vector<Node>> all_paths; //
    std::vector<std::vector<std::vector<Node>>> planned_paths_per_agent;
    std::vector<std::vector<Node>> assigned_paths; // the 'optimally' assigned paths
    std::vector<std::tuple<int, int>> allocated_tasks_optimized;

    cv::Mat image; // I think we can make it here to make our life easier, we could probably recreate it after if needed
    cv::Mat image_scaled_up;
};

struct data_to_save
{
    simulation_info sim_info;
    std::vector<double> filtering_values;
    std::vector<data_one_step> result;
};


void generate_sampled_images(std::string map_location, std::string target_folder, int num_start_locations, int num_target_locations, size_t num_samples);




void draw_pixels_to_image(cv::Mat& image, std::vector<Node> nodes, const cv::Vec3b& color, double opacity = 1);
std::vector<Node> sample_free_points_on_map(const std::vector<std::vector<int>>& map, const int number_of_samples);
std::vector<std::vector<int>> image_to_occupancy_grid(std::string file);
cv::Mat occupancy_grid_to_image(const std::vector<std::vector<int>>& grid);

void draw_circle_to_image(cv::Mat& image, size_t scaling_factor, std::vector<Node> path, cv::Vec3b color, int thickness, double radius);
std::vector<Node> nodes_from_image(const cv::Mat& image, const cv::Vec3b& color);

bool compare_color(const cv::Vec3b& c1, const cv::Vec3b& c2, const size_t& max_distance = 10);


double path_to_distance(const std::vector<Node>& path);



// double path_to_distance(const std::vector<Node>& path);
double find_target_radius(const Node& start_location, const std::vector<Node>& target_locations, const double percentage);


// std::vector<std::vector<double>> generate_cost_vector(const std::vector<Node>& start_points, const std::vector<Node>& target_points, const std::vector<std::vector<Node>>& all_paths);









// save to csv files
void test_save_file(const data_to_save& save, const std::string& directory);


// class Simulation_2d //: public rclcpp::Node
// {
//     private:

//     public:
//     Simulation_2d();
//     ~Simulation_2d();

// };




void map_to_world_size(std::vector<Node>& path_vector, double resolution_meter_per_pixel, double map_origin_x, double map_origin_y);
Node world_to_map(const Node& point, double resolution_meter_per_pixel, double map_origin_x, double map_origin_y);



// bool operator==(const std::vector<std::tuple<int, int>>& a, const std::vector<std::tuple<int, int>>& b)
// {
//     // check if same size
//     if(a.size() != b.size())
//     {
//         return false;
//     }

//     bool return_value = true;
//     for(size_t i = 0; i < a.size(); ++i)
//     {
//         if(a[i] != b[i])
//         {
//             return false;
//         }
//     }

//     return return_value;
// }



} // end namespace simulation_2d









#endif
