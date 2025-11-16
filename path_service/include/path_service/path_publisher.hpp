


#ifndef PATH_PUBLISHER_HPP
#define PATH_PUBLISHER_HPP


#include "rclcpp/rclcpp.hpp"
#include "path_service/srv/get_path.hpp"
#include "path_service/srv/set_map.hpp"
#include "path_service/srv/position_status.hpp"


#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <unistd.h> // linux
#include <sys/stat.h>
#include <filesystem>
#include "yaml-cpp/yaml.h"


#include "simulation_2d.hpp"




// function outside class to avoid namespace issues... (path planning Node and ros Node)
bool get_path(const std::vector<std::vector<int>>& global_grid, nav_msgs::msg::Path& planned_path, const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& end_point, double resolution_meter_per_pixel, double map_origin_x, double map_origin_y)
{
    std::vector<geometry_msgs::msg::PoseStamped> path_poses;

    Node start;
    Node target;


    // transform to map coordinates
    double x_start_transformed = (start_point.x - map_origin_x) / resolution_meter_per_pixel;
    double y_start_transformed = (start_point.y - map_origin_y) / resolution_meter_per_pixel;

    double x_target_transformed = (end_point.x - map_origin_x) / resolution_meter_per_pixel;
    double y_target_transformed = (end_point.y - map_origin_y) / resolution_meter_per_pixel;


    start.x_ = x_start_transformed;
    start.y_ = y_start_transformed;

    target.x_ = x_target_transformed;
    target.y_ = y_target_transformed;



    // transform to map coordinates
    // start = simulation_2d::world_to_map(start, resolution_meter_per_pixel, map_origin_x, map_origin_y);
    // target = simulation_2d::world_to_map(target, resolution_meter_per_pixel, map_origin_x, map_origin_y);


    // TODO: check that points are inside map to avoid segmentation fault here...
    // plan path
    DStarLite d_star_lite(global_grid);
    auto [path_found, path_vector] = d_star_lite.Plan(start, target);
    if(path_found == false)
    {
        std::cout << "PATH NOT FOUND? (" << start_point.x << ", " << start_point.y << ")->(" << end_point.x << ", " << end_point.y << ")\n";
        return false;
    }

    // transform back to world coordinates
    simulation_2d::map_to_world_size(path_vector, resolution_meter_per_pixel, map_origin_x, map_origin_y);


    path_poses.reserve(path_vector.size()) ;
    // convert to nav_msgs::msg::Path
    for(const Node& node : path_vector)
    {
        // TODO: optimize this?
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = node.x_ + 0.5 * resolution_meter_per_pixel;
        pose.pose.position.y = node.y_ + 0.5 * resolution_meter_per_pixel;
        path_poses.push_back(pose);
        // std::cout << "Point: (" << pose.pose.position.x << ", " << pose.pose.position.y << ")  \n";
    }

    planned_path.poses = path_poses;

    // std::cout << "Start: (" << start.x_ << ", " << start.y_ << "). End: (" << target.x_ << ", " << target.y_  << "). length: " << path_poses.size() << ".\n"; 
    // std::cout << "Start: (" << path_vector[0].x_ << ", " << path_vector[0].y_ << "). End: (" << path_vector[path_vector.size() - 1].x_ << ", " << path_vector[path_vector.size() - 1].y_  << "). length: " << path_poses.size() << ".\n"; 
    return true;
}



bool point_is_free(const std::vector<std::vector<int>>& global_grid, const geometry_msgs::msg::Point& target_point, double resolution_meter_per_pixel, double map_origin_x, double map_origin_y)
{
    // transform to map coordinates
    double x_transformed = (target_point.x - map_origin_x) / resolution_meter_per_pixel;
    double y_transformed = (target_point.y - map_origin_y) / resolution_meter_per_pixel;

    // std::cout << "Map status at (" << target_point.x << ", " << target_point.y << "): " <<  global_grid[x_transformed][y_transformed] << "\n";

    // Map state == 1 -> occupied
    // Map state == 0 -> free
    if(global_grid[x_transformed][y_transformed] == 1)
    {
        return false;
    }
    else
    {
        return true;
    }
}




















class Path_generator : public rclcpp::Node
{
    public:
    Path_generator();
    ~Path_generator();


    private:
    rclcpp::TimerBase::SharedPtr some_timer;
    nav_msgs::msg::Odometry current_odom;
    bool odom_received;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_position_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;


    rclcpp::Service<path_service::srv::GetPath>::SharedPtr path_cost_service;
    rclcpp::Service<path_service::srv::SetMap>::SharedPtr update_map_path_service;
    rclcpp::Service<path_service::srv::PositionStatus>::SharedPtr get_map_position_service;

    void init_map_to_grid(std::string path);

    void odometry_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void goal_position_callback(geometry_msgs::msg::Point::SharedPtr msg);


    void get_map_position_status( std::shared_ptr<path_service::srv::PositionStatus::Request> request, std::shared_ptr<path_service::srv::PositionStatus::Response> response);
    void plan_path_service( std::shared_ptr<path_service::srv::GetPath::Request> request, std::shared_ptr<path_service::srv::GetPath::Response> response);
    void set_map_by_path( std::shared_ptr<path_service::srv::SetMap::Request> request, std::shared_ptr<path_service::srv::SetMap::Response> response);
    double path_to_distance(const nav_msgs::msg::Path& path);


    std::vector<std::vector<int>> global_grid;


    // offset in meter. Tells how the map is moved in relation to the world
    double resolution_meter_per_pixel;
    double map_origin_x;
    double map_origin_y;
};

Path_generator::Path_generator() : Node("Path_generator")
{
    // RCLCPP_INFO_STREAM(this->get_logger(), "");
    rclcpp::QoS qos = rclcpp::QoS(1);
    qos.reliable();
    qos.keep_last(1);

    this->goal_position_subscriber = this->create_subscription<geometry_msgs::msg::Point>("path_planning/goal", qos, std::bind(&Path_generator::goal_position_callback, this, std::placeholders::_1));
    this->odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odometry", qos, std::bind(&Path_generator::odometry_callback, this, std::placeholders::_1));

    this->path_publisher = this->create_publisher<nav_msgs::msg::Path>("path_planning/path", qos);

    this->path_cost_service = this->create_service<path_service::srv::GetPath>("get_path", std::bind(&Path_generator::plan_path_service, this, std::placeholders::_1, std::placeholders::_2));
    this->update_map_path_service = this->create_service<path_service::srv::SetMap>("set_map_filepath", std::bind(&Path_generator::set_map_by_path, this, std::placeholders::_1, std::placeholders::_2));
    this->get_map_position_service = this->create_service<path_service::srv::PositionStatus>("get_map_position_status", std::bind(&Path_generator::get_map_position_status, this, std::placeholders::_1, std::placeholders::_2));



    // get ROS parameters
    try
    {
        this->declare_parameter("map_url", "/home/niklas/ws/ros_auction_ws/src/planning/path_service/map/map_safety.yaml"); // Default value
    }
    catch(const std::exception& e) { }
    std::string map_file_path;
    this->get_parameter("map_url", map_file_path);

    init_map_to_grid(map_file_path);
}

Path_generator::~Path_generator()
{

}

void Path_generator::init_map_to_grid(std::string path)
{
    // Check if file exists
    struct stat buffer;
    if(stat (path.c_str(), &buffer) != 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Map file '" << path << "' does not exist");
        throw std::invalid_argument("Map file does not exists!");
    }

    // init map parameters from the yaml file
    std::string image_name;
    double resolution;
    std::vector<double> origin;
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        image_name = config["image"].as<std::string>();
        resolution = config["resolution"].as<double>();
        origin = config["origin"].as<std::vector<double>>();

        this->resolution_meter_per_pixel = resolution;
        this->map_origin_x = origin[0];
        this->map_origin_y = origin[1];
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Something is wrong with reading the yaml file. Did you specify a yaml file as the ros argument?");
        std::cerr << e.what() << '\n';
        return;
    }
    
    // TODO: read the other yaml parameters as well? 

    

    std::filesystem::path p(path);
    std::string directiory = p.parent_path();
    // init path planner here
    global_grid = simulation_2d::image_to_occupancy_grid(p.parent_path().string() + "/" + image_name);
    RCLCPP_INFO_STREAM(this->get_logger(), "Path planner initialized with image " << path << ". Origin: (" << this->map_origin_x << ", " << this->map_origin_y << ") Res: " << this->resolution_meter_per_pixel);

    // image_map = cv::imread(path);
}



void Path_generator::odometry_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    this->odom_received = true;
    this->current_odom = *msg.get();
}
void Path_generator::goal_position_callback(geometry_msgs::msg::Point::SharedPtr msg)
{
    if(this->odom_received == false)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "No odometry received, can't plan path.");
        return;
    }
    
    
    // plan path to goal in msg

    geometry_msgs::msg::Point current_point;
    geometry_msgs::msg::Point target_point;

    current_point = this->current_odom.pose.pose.position;
    target_point = *msg.get();


    nav_msgs::msg::Path planned_path;
    get_path(this->global_grid, planned_path, current_point, target_point, this->resolution_meter_per_pixel, this->map_origin_x, this->map_origin_y);


    // add header
    planned_path.header.frame_id = "world";
    planned_path.header.stamp = this->get_clock()->now();

    // publish
    this->path_publisher.get()->publish(planned_path);
    return;
}














double Path_generator::path_to_distance(const nav_msgs::msg::Path& path)
{
    bool first_iteration = true;
    double total_distance = 0;
     geometry_msgs::msg::PoseStamped previous_pose;
    for(const geometry_msgs::msg::PoseStamped& pose : path.poses)
    {
        if(first_iteration == true)
        {
            previous_pose = pose;
            first_iteration = false;
            continue;
        }
        
        total_distance += std::sqrt(std::pow(previous_pose.pose.position.x - pose.pose.position.x, 2) + 
                                    std::pow(previous_pose.pose.position.y - pose.pose.position.y, 2));
        previous_pose = pose;
    }
    
    return total_distance;
}





//////////////// SERVICE //////////////

void Path_generator::plan_path_service( std::shared_ptr<path_service::srv::GetPath::Request> request, std::shared_ptr<path_service::srv::GetPath::Response> response)
{
std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
(void) begin;



    geometry_msgs::msg::Point current_point;
    geometry_msgs::msg::Point target_point;

    current_point = request.get()->start_position;
    target_point = request.get()->goal_position;


    nav_msgs::msg::Path planned_path;
    bool path_found = get_path(this->global_grid, planned_path, current_point, target_point, this->resolution_meter_per_pixel, this->map_origin_x, this->map_origin_y);


    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    (void) end;


    float distance = this->path_to_distance(planned_path);
    std::cout << "Computation time (path planning)  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << ". Distance: " << distance << std::endl;


    // add header
    planned_path.header.frame_id = "world";
    planned_path.header.stamp = this->get_clock()->now();



    response.get()->path_cost = distance;
    response.get()->path = planned_path;
    response.get()->path_found = path_found;
    // response.get()->computation_time = ...; // TODO


    return;
}



void Path_generator::set_map_by_path( std::shared_ptr<path_service::srv::SetMap::Request> request, std::shared_ptr<path_service::srv::SetMap::Response> response)
{
    if(access(request.get()->map_file_path.c_str(), 0) == -1)
    {
        response.get()->success = false;
        return;    
    }


    std::cout << "GOT NEW MAP" << request.get()->map_file_path << "\n";

    this->init_map_to_grid(request.get()->map_file_path);
    response.get()->success = true;
    return;
}



void Path_generator::get_map_position_status( std::shared_ptr<path_service::srv::PositionStatus::Request> request, std::shared_ptr<path_service::srv::PositionStatus::Response> response)
{
    geometry_msgs::msg::Point target_point = request.get()->map_position;

    bool map_point_is_free = point_is_free(this->global_grid, target_point, this->resolution_meter_per_pixel, this->map_origin_x, this->map_origin_y);
    
    response.get()->position_is_free = map_point_is_free;
    return;
}


#endif
