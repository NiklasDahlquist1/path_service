#include "rclcpp/rclcpp.hpp"


#include "path_service/simulation_2d.hpp"




#include "path_service/srv/get_path.hpp"


#include "path_service/path_publisher.hpp"

#include <memory>


#include <chrono>




// //


// TMP, use the class later

// std::string file_path_map = "/home/niklas/ws/ros_auction_ws/src/path_service/map/map.png";
std::string file_path_map = "/home/niklas/ws/ros_auction_ws/src/path_service/map/map.png";
std::vector<std::vector<int>> global_grid;
cv::Mat image_map;
cv::Mat image_annotated;


const float resolution_meter_per_pixel = 1;//1.64;
const float map_origin_x = 0; // offset in meter. Tells how the map is moved in relation to the world
const float map_origin_y = 0;


// void map_to_world_size(std::vector<Node>& path_vector)
// {
//     for(Node& n : path_vector)
//     {
//         n.x_ = n.x_ * resolution_meter_per_pixel;
//         n.y_ = n.y_ * resolution_meter_per_pixel;
        
//         n.x_ = n.x_ - map_origin_x;
//         n.y_ = n.y_ - map_origin_y;
//     }
//     return;
// }

// Node world_to_map(const Node& point)
// {
//     Node transformed;

//     transformed.x_ = transformed.x_ + map_origin_x;
//     transformed.y_ = transformed.y_ + map_origin_y;

//     transformed.x_ = point.x_ / resolution_meter_per_pixel;
//     transformed.y_ = point.y_ / resolution_meter_per_pixel;

//     return transformed;
// }




void plan_path_service( std::shared_ptr<path_service::srv::GetPath::Request> request, std::shared_ptr<path_service::srv::GetPath::Response> response)
{
std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
(void) begin;
    
    // geometry_msgs::msg::Point start_position = request.get()->start_position;
    // geometry_msgs::msg::Point goal_position = request.get()->goal_position;
    std::vector<geometry_msgs::msg::PoseStamped> path_poses;

    // cv::Mat image_display;
    // cv::Mat image = cv::imread("/home/niklas/ws/ros_auction_ws/src/path_service/map/map.png");
    Node start;
    Node target;

    start.x_ = request.get()->start_position.x;
    start.y_ = request.get()->start_position.y;

    target.x_ = request.get()->goal_position.x;
    target.y_ = request.get()->goal_position.y;


    // transform to map coordinates
    start = simulation_2d::world_to_map(start, resolution_meter_per_pixel, map_origin_x, map_origin_y);
    target = simulation_2d::world_to_map(target, resolution_meter_per_pixel, map_origin_x, map_origin_y);


    // plan path
    DStarLite d_star_lite(global_grid);
    auto [path_found, path_vector] = d_star_lite.Plan(start, target);
    if(path_found == false)
    {
        std::cout << "PATH NOT FOUND? status: " << path_found << "\n";
        response.get()->path_cost = -1;
        response.get()->path.poses = path_poses;
    }
    




std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
(void) end;
// std::cout << "Time for path planning = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;


// Visualize for debugging

    // if(false)
    // {
    //     image_annotated = image_map.clone();
    //     simulation_2d::draw_pixels_to_image(image_annotated, path_vector, {1, 255, 0}, 1);
    //     cv::Mat test_image = simulation_2d::occupancy_grid_to_image(global_grid);

    //     cv::imwrite("/tmp/images/image_annotated.png", image_annotated);
    //     cv::imwrite("/tmp/images/image_map.png", image_map);
    //     cv::imwrite("/tmp/images/image_test.png", test_image);
    // }










    // transform back to world coordinates
    simulation_2d::map_to_world_size(path_vector, resolution_meter_per_pixel, map_origin_x, map_origin_y);

    float distance = simulation_2d::path_to_distance(path_vector);
    for(const Node& node : path_vector)
    {
        // TODO: optimize this
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = node.x_;
        pose.pose.position.y = node.y_;
        path_poses.push_back(pose);
    }



    // cv::imshow("My Image", image_annotated);
    // cv::waitKey(20);


            // draw_pixels_to_image(image, path_vector, {(uchar) distr(eng), (uchar) distr(eng), (uchar) distr(eng)});
            // cv::circle(image, cv::Point(start.y_, start.x_),max_distance, cv::Scalar(0,0,255), 1, 8,0); //cv::FILLED


            // // visualization
            // draw_pixels_to_image(image_tmp, path_vector, {(uchar) distr(eng), (uchar) distr(eng), (uchar) distr(eng)});
            // draw_pixels_to_image(image_tmp, start_points, {0, 255, 0});
            // draw_pixels_to_image(image_tmp, goal_points, {0, 0, 255});
            // //cv::circle(image_tmp, cv::Point(start.y_, start.x_),max_distance, cv::Scalar(0,0,255), 1, 8,0); //cv::FILLED

            // // show the image on window
            // cv::resize(image_tmp, image_display, cv::Size(image_tmp.cols * 16, image_tmp.rows * 16), 0, 0, cv::INTER_AREA);
            // cv::circle(image_display, cv::Point(start.y_*16, start.x_*16),max_distance*16, cv::Scalar(0,0,255), 1*16, 8,0); //cv::FILLED
            
            // draw_circle_to_image(image_display, 16, path_vector, {50, 200, 50}, cv::FILLED, 1.1);
            // draw_circle_to_image(image_display, 16, start_points, constants::starting_location, cv::FILLED, 1.6);
            // draw_circle_to_image(image_display, 16, goal_points, constants::target_location, cv::FILLED, 1.6);
            

            // cv::imshow("My Image", image_display);
            // cv::waitKey(20);





//     /*
//         - use local map to plan path
//         - should we have a service to update map?
//         - ...
//     */ 
    

    
    response.get()->path_cost = distance;
    response.get()->path.poses = path_poses;


// end = std::chrono::steady_clock::now();
// std::cout << "Total time (including saving images) = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;



    return;
}








int main(int argc, char **argv)
{
    // init global variables
    global_grid = simulation_2d::image_to_occupancy_grid(file_path_map); // TODO: Dont use this global path...
    image_map = cv::imread(file_path_map);





    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("path_service");
    // rclcpp::CallbackGroup::SharedPtr callback_group_service_call = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // rclcpp::Service<path_service::srv::GetPath>::SharedPtr cost_service = node->create_service<path_service::srv::GetPath>("get_path", &plan_path_service);

    /* TODO:
        - Integrate the get_path service into the path publisher class (to make sure they share the same map, etc)
        - Add a service to load a new map during execution (to 'allow for map updates')
    */


    std::shared_ptr<rclcpp::Node> node_path_generator = std::make_shared<Path_generator>();



    // //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service ready.");

    // rclcpp::spin(node);
    rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(node);
    exec.add_node(node_path_generator);
    exec.spin();


    rclcpp::shutdown();
    return 0;
}
