
#include "path_service/simulation_2d.hpp"

// #include "simulation_2d/optimization_ortools.h"



namespace simulation_2d
{


cv::Vec3b blend_pixel(const cv::Vec3b& source_color, const cv::Vec3b& overlay_color, double opacity)
{
    // https://stackoverflow.com/questions/726549/algorithm-for-additive-color-mixing-for-rgb-values

    if(opacity > 0.99) // just to be safe
    {
        return overlay_color;
    }
    double alpha_fg = opacity;
    double alpha_bg = 1;
    double new_alpha = 1 - (1 - alpha_fg) * (1 - alpha_bg);
    uint8_t r = (uint8_t) (overlay_color[2] * alpha_fg / new_alpha + source_color[2] * alpha_bg * (1 - alpha_fg) / new_alpha);
    uint8_t g = (uint8_t) (overlay_color[1] * alpha_fg / new_alpha + source_color[1] * alpha_bg * (1 - alpha_fg) / new_alpha);
    uint8_t b = (uint8_t) (overlay_color[0] * alpha_fg / new_alpha + source_color[0] * alpha_bg * (1 - alpha_fg) / new_alpha);
    return {b, g, r};

}

// should be updated
void draw_pixels_to_image(cv::Mat& image, std::vector<Node> nodes, const cv::Vec3b& color, double opacity)
{
    for(Node& n : nodes)
    {
        size_t y_image = image.size().height - n.y_ - 1;
        cv::Vec3b image_current_color = image.at<cv::Vec3b>(y_image, n.x_);
        // image.at<cv::Vec3b>(n.y_, n.x_) = color;
        image.at<cv::Vec3b>(y_image, n.x_) = blend_pixel(image_current_color, color, opacity);
        //image[n.y_, n.x_, 2] = (uint8_t) 255;
        // .at<uint8_t>(n.x_,n.y_)
    }
    
    return;
}



// WARNING
std::vector<Node> sample_free_points_on_map(const std::vector<std::vector<int>>& map, const int number_of_samples)
{
    std::vector<std::vector<int>> map_copy = map;
    int sampled = 0;

    std::random_device rd;  
    std::mt19937 eng(rd());  
    std::uniform_int_distribution<int> distr_row(0, map[0].size() - 1);  
    std::uniform_int_distribution<int> distr_col(0, map.size() - 1);  
    // std::normal_distribution<double> distr_col(160, 80.0);  
    // std::normal_distribution<double> distr_row(160, 80.0);  

    std::vector<Node> points;

    while(sampled < number_of_samples)
    {
        // generate random point
        int x = (int) distr_col(eng);
        int y = (int) distr_row(eng);
        // std::cout << "number: " << x << ", " << y << "\n";

        // check so that the point is actually inside the map (might be needed depending on the distribution)
        if(x < 0 || x > (int) map_copy.size() ||
           y < 0 || y > (int) map_copy[0].size())
        {
            continue;
        }

        // check so that it is not inside a wall or on the same positions as a previous point
        if(map_copy[x][y] == 0)
        {
            Node node(x = x, y = y);
            points.push_back(node);
            sampled += 1;        
            map_copy[x][y] = 1;
        }
    }
    // std::cout << "Sampled " << sampled << " number of points\n";

    return points;
}



// should be updated
cv::Mat occupancy_grid_to_image(const std::vector<std::vector<int>>& grid)
{
    cv::Mat image(grid[0].size(), grid.size(), CV_8UC3);//(CV_MAT_DEPTH(0) + (((3)-1) << CV_CN_SHIFT)));

    for (size_t y = 0; y < grid[0].size(); ++y)
    {
        for (size_t x = 0; x < grid.size(); ++x)
        {
            size_t y_image = grid[0].size() - y - 1;
            cv::Vec3b& pixel_value = image.at<cv::Vec3b>(y_image,x); // needs to be initialized, for now just set it to an existing pixel value...
            if(grid[x][y] == 1)
            {
                pixel_value = constants::occupied;
            }
            else
            {
                pixel_value = constants::free;
            }
            

            image.at<cv::Vec3b>(y_image,x) = pixel_value;
        }
    }

    return image;
}



// should be updated
std::vector<std::vector<int>> image_to_occupancy_grid(std::string file)
{
    cv::Mat image_color = cv::imread(file);
    if(image_color.data == NULL) std::cout << "CANT FIND FILE " << file << " when converting to occupancy grid\n";

    cv::Mat image_grey;
    cv::cvtColor(image_color, image_grey, cv::COLOR_BGR2GRAY);


    int cols = image_color.cols;
    int rows = image_color.rows;
    // std::cout << "Image dimensions (col, row): (" << cols << ", " << rows << ")\n";
    std::vector<std::vector<int>> map_grid(cols, std::vector<int>(rows, 0));
    // std::cout << "Occupancy dimensions (x, y): (" << map_grid.size() << ", " << map_grid[0].size() << ")\n";



    // cv::namedWindow("My Image", cv::WINDOW_NORMAL);
    // cv::resizeWindow("My Image", 1700, 1700);

    // std::cout << "showing color image: \n";
    // cv::imshow("My Image", image_color);
    // cv::waitKey(10000);
    // std::cout << "showing greyscale image: \n";
    // cv::imshow("My Image", image_grey);
    // cv::waitKey(10000);


    for(int x = 0; x < cols; ++x)
    {
        for(int y_image = 0; y_image < rows; ++y_image)
        {
            int y = rows - y_image - 1;
            // cv::Scalar_<uint8_t> pixel_value = image_color.at<cv::Scalar_<uint8_t>>(i,j);
            // cv::Vec3b pixel_occupied = {(uchar) pixel_value[0], (uchar) pixel_value[1], (uchar) pixel_value[2]};
            cv::Vec3b pixel_value = image_color.at<cv::Vec3b>(y_image, x); //
            
            // std::cout << "Pixel value: " << pixel_value << "\n";

            if(compare_color(pixel_value, constants::occupied, 100) == true)
            {
                // std::cout << "Position (" << x << ", " << y << "), value: " << "1. Color: " << "(" << pixel_value << "\n";
                map_grid[x][y] = 1;
            }
            else
            {
                // std::cout << "Position (" << x << ", " << y << "), value: " << "1. Color: " << "(" << pixel_value << "\n";
                map_grid[x][y] = 0;
            }
            // map[j][i] = 1 - (int) image_grey.at<uint8_t>(i,j) / 255;
            // std::cout << "pixel (" << i << ", " << j << "), value: " << map[i][j] << " Grey: " << "(" << (int)image_grey.at<uint8_t>(i,j) <<")\n";
            // std::cout << "-----\n";
            // if(map[i][j] != map_test[i][j])
            // {
            //     std::cout << "pixel (" << i << ", " << j << ") differs..." << "\n";
            //     std::cout << "Geryscale image, value: " << "(" << (int)image_grey.at<uint8_t>(i,j) <<")\n";
            //     std::cout << "color image, value: " << "(" << image_color.at<cv::Vec3b>(i,j) <<")\n";
            // }
        }
    }

    // std::cout << "identical?: " << (map == map_test) << "\n"; // will only work for a pure black and white





    // for(size_t i = 0; i < map_test.size(); ++i)
    // {
    //     std::cout << "row: ";
    //     for(size_t j = 0; j < map_test[0].size(); ++j)
    //     {
    //         std::cout << map_test[i][j] << ",";
    //     }
    //     std::cout << "\n";
    // }


    return map_grid;

}


// checks the 'distance' between two colors. Not great way to compare colors TODO: remake this
bool compare_color(const cv::Vec3b& c1, const cv::Vec3b& c2, const size_t& max_distance)
{ 
    return (std::pow(c1[0] - c2[0], 2) + 
            std::pow(c1[1] - c2[1], 2) +
            std::pow(c1[2] - c2[2], 2))
            < max_distance*max_distance;
}



// WARNING
// void draw_circle_to_image(cv::Mat& image, size_t scaling_factor, std::vector<Node> path, uint8_t r, uint8_t g, uint8_t b, int thickness, double radius)
void draw_circle_to_image(cv::Mat& image, size_t scaling_factor, std::vector<Node> path, cv::Vec3b color, int thickness, double radius)
{
    // cv::Mat overlay = image.clone();
    for(Node& n : path)
    {
        //cv::circle(overlay, cv::Point(n.x_ * scaling_factor + scaling_factor / 2, n.y_ * scaling_factor + scaling_factor / 2), radius * scaling_factor, cv::Scalar_<uint8_t>(color[0],color[1],color[2]), thickness, 0); //
        cv::circle(image, cv::Point(n.x_ * scaling_factor + scaling_factor / 2, n.y_ * scaling_factor + scaling_factor / 2), radius * scaling_factor, cv::Scalar_<uint8_t>(color[0],color[1],color[2]), thickness, cv::LINE_AA); //
    }
    // cv::addWeighted(overlay, 0.5, image, 1 - 0.5, 0, image);
}

std::vector<Node> nodes_from_image(const cv::Mat& image, const cv::Vec3b& color)
{
    int cols = image.cols;
    int rows = image.rows;
    std::vector<Node> nodes;

    for(int x = 0; x < cols; ++x)
    {
        for(int y_image = 0; y_image < rows; ++y_image)
        {
            int y = rows - y_image - 1;
            
            // if(image.at<cv::Vec3b>(i,j) == color)
            if(compare_color(image.at<cv::Vec3b>(y_image,x), color) == true)
            {
                Node n;
                n.x_ = x;
                n.y_ = y;
                nodes.push_back(n);
                std::cout << "node at position (" << x << ", " << y << ")\n";
            }
            // std::cout << "pixel (" << i << ", " << j << "), value: " << image.at<cv::Scalar>(i,j) << "\n";
        }
    }

    return nodes;
}





double path_to_distance(const std::vector<Node>& path)
{
    bool first_iteration = true;
    double total_distance = 0;
    Node last_node;
    for(const Node& node : path)
    {
        if(first_iteration == true)
        {
            last_node = node;
            first_iteration = false;
            continue;
        }
        
        total_distance += std::sqrt(std::pow(node.x_ - last_node.x_, 2) + 
                                    std::pow(node.y_ - last_node.y_, 2));
        last_node = node;
    }
    
    return total_distance;
}








// find the target radius that includes the requested percentage of tasks (rounded to closest integer value)
double find_target_radius(const Node& start_location, const std::vector<Node>& target_locations, const double percentage)
{
    
    std::vector<std::tuple<double, Node>> target_locations_sorted; // target locations and distance squared
    for(const Node& target : target_locations)
    {
        double distance2 = std::pow(target.x_ - start_location.x_, 2) + 
                           std::pow(target.y_ - start_location.y_, 2);
        std::tuple<double, Node> tuple = std::make_tuple(distance2, target);
        target_locations_sorted.push_back(tuple);

    }
    // sort target locations

    std::sort(std::begin(target_locations_sorted), std::end(target_locations_sorted), 
        [](std::tuple<double, Node> const &t1, std::tuple<double, Node> const &t2) {
            return std::get<0>(t1) < std::get<0>(t2); // or use a custom compare function
        }
    );



    // for(auto a : target_locations_sorted)
    // {
    //     std::cout << std::get<0>(a) << " ";
    // }
    // std::cout << "\n";

    // find what index corresponds to the correct percentage
    int locations_needed = std::round(target_locations_sorted.size() * percentage);
    // std::cout << "Number of tasks: " << target_locations.size() << ". tasks needed: " << locations_needed << ". share of tasks: " << (double) locations_needed / target_locations.size() << ". giving radius: " << std::sqrt(std::get<0>(target_locations_sorted[locations_needed - 1])) << "\n";

    // calculate radius for that target location
    return std::sqrt(std::get<0>(target_locations_sorted[locations_needed - 1]));
}








// WARNING
void generate_sampled_images(std::string map_location, std::string target_folder, int num_start_locations, int num_target_locations, size_t num_samples)
{
    std::cout << "starting map sampling\n";
    std::filesystem::create_directories(target_folder);
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(size_t i = 0; i < num_samples; ++i)
    {
        std::string current_directiory = target_folder + "map_" + std::to_string(i) + "/";
        std::filesystem::create_directories(current_directiory);

        auto grid = image_to_occupancy_grid(map_location);
        // cv::Mat image = cv::imread(map_location);
        auto start_points = sample_free_points_on_map(grid, num_start_locations);
        auto goal_points = sample_free_points_on_map(grid, num_target_locations);

        cv::Mat image = occupancy_grid_to_image(grid);
        
        // cv::namedWindow("My Image", cv::WINDOW_NORMAL);
        // cv::resizeWindow("My Image", 1700, 1700);
        // std::cout << "showing occupied walls: \n";
        // cv::imshow("My Image", image_occupied);
        // cv::waitKey(10000);
        cv::imwrite(current_directiory + "map_occupancy"+ ".png", image);
        
        draw_pixels_to_image(image, start_points, constants::starting_location);
        draw_pixels_to_image(image, goal_points, constants::target_location);

        // std::cout << "showing map with start/target locations: \n";
        // cv::imshow("My Image", image_occupied);
        // cv::waitKey(10000);
        
        cv::imwrite(current_directiory + "map_sampled_locations" + ".png", image);


        // visualization
        cv::Mat image_display;
        size_t scale_factor = 16;
        cv::resize(image, image_display, cv::Size(image.cols * scale_factor, image.rows * scale_factor), 0, 0, cv::INTER_AREA);
        
        draw_circle_to_image(image_display, scale_factor, start_points, constants::starting_location, cv::FILLED, 1.6);
        draw_circle_to_image(image_display, scale_factor, goal_points, constants::target_location, cv::FILLED, 1.6);
        cv::imwrite(current_directiory + "map_scaled_up" + ".png", image_display);

    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time sampling map spent " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " milliseconds" << "\n"; 
}





















void map_to_world_size(std::vector<Node>& path_vector, double resolution_meter_per_pixel, double map_origin_x, double map_origin_y)
{
    for(Node& n : path_vector)
    {
        n.x_ = n.x_ * resolution_meter_per_pixel;
        n.y_ = n.y_ * resolution_meter_per_pixel;
        
        n.x_ = n.x_ + map_origin_x;
        n.y_ = n.y_ + map_origin_y;
    }
    return;
}

Node world_to_map(const Node& point, double resolution_meter_per_pixel, double map_origin_x, double map_origin_y)
{
    // TODO: some error occur due to integers... fix please
    Node transformed;

    transformed.x_ = point.x_ - map_origin_x;
    transformed.y_ = point.y_ - map_origin_y;

    transformed.x_ = transformed.x_ / resolution_meter_per_pixel;
    transformed.y_ = transformed.y_ / resolution_meter_per_pixel;

    return transformed;
}






// std::vector<std::vector<double>> generate_cost_vector(const std::vector<Node>& start_points, const std::vector<Node>& target_points, const std::vector<std::vector<Node>>& all_paths)
// {

// }











// Simulation_2d::Simulation_2d() 
// {

// }


// Simulation_2d::~Simulation_2d()
// {

// }





} // end namespace simulation_2d








