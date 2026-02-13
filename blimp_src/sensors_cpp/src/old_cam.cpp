
//std::cout << "Detected Goal - Max X: " << max_x << ", Min X: " << min_X << ", Max Y: " << max_y << ", Min Y: "<< min_y< < std::endl;// dependicies and libraires needed for detect_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/camera_coord.hpp"
#include "blimp_interfaces/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencv2/opencv.hpp" // inlcluding class for capturing from video files, image sequences or cameras
#include "vector"
#include "iostream"

// creating class with name CamNode
// ros2 Notation foor creatin node in public using rclpp ros2 communication matrix
class CamNode : public rclcpp::Node
{
public: 
    CamNode() : Node("cam_node") 
    {
	// creating publisher, publishing on the topic "cam_data" 
        cam_data_publisher_ = this->create_publisher<blimp_interfaces::msg::CameraCoord>("cam_data", 3);
	cam_flag_publisher_ = this->create_publisher<blimp_interfaces::msg::Bool>("cam_flag",3);
	// subcribsing to the topic "joy"
        subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&CamNode::callback_read_joy, this, std::placeholders::_1));
        // Subsribing to net servo topic
        subscriber_net_servo = this->create_subscription<blimp_interfaces::msg::Bool>(
            "net_flag", 10, std::bind(&CamNode::callback_read_net, this, std::placeholders::_1));
	// setting variable cap_ to default constructer VideoCapture, CAP_V4L2 sets the cap to the proper video channel for linux
        cap_ = cv::VideoCapture(0, cv::CAP_V4L2);
	// setting frame width of pi camera
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280); //640 1280
	// setting frame height of py camera
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720); //480 720
        frame_count_ = 0; 
        minimum_radius_ = 5; // setting minimum radius that camera detects (eliminating false positives)
        findGoal = true; // Flag for switching between goal detection and balloon detection, this is used for testing
	goal_flag = false;
        net_flag = false;
	// Parameters for HoughLinesP
        rho = 1; 
        theta = CV_PI / 180;
        threshold = 75;
        min_line_length = 50; // minimum line length for goal detection
        max_line_gap = 30; // maximum line gap for goal detection
        
	    
	cam_mode = true; // Flag for goal detection and balloon detection
        total_lines = 0; // Counter for goal detection averaging, this used in our code
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CamNode::callback_read_image, this));
        RCLCPP_INFO(this->get_logger(), "Video Detection has Started, press w to switch detection");
    }

public:
    
    // funtion to read xbox buttons in order to switch between goal and balloon detection
    int counter_timer = 0;
    void callback_read_joy(const sensor_msgs::msg::Joy::SharedPtr button)
    {
        x_button = button->buttons[3];
    }
    void callback_read_net(const blimp_interfaces::msg::Bool &msg)
    {
    net_flag = msg.flag;  // Access the 'flag' attribute from the message
    RCLCPP_INFO(this->get_logger(), "Received net_flag: %s", net_flag ? "true" : "false");
    }
    void callback_read_image() {
        if ((x_button == 1)) {
            cam_mode = !cam_mode;
	    x_button = 0;
            sleep(1);
            RCLCPP_INFO(this->get_logger(), "Cam Mode has been Switched");
        }

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not open video source.");
            return;
        }

        counter_timer++;
	
	// Test timer to switch modes
        if (counter_timer == 25){
                counter_timer = 0;
               findGoal = !findGoal;
               /* if(findGoal){
                       RCLCPP_INFO(this->get_logger(), "Goal Detection has started!");
                } else {
                        RCLCPP_INFO(this->get_logger(), "Balloon Detection has stated!");
                }*/
        }

	// Making separate frame matrices to run both detection's respective methods without affecting the other detection
	cv::Mat frame, goal;
        cap_ >> frame;
        cap_ >> goal;

	//Checking frame is readable
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
            return;
        }

	//HSV matrices to store the color filtering
        cv::Mat hsv_frame, goal_hsv;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::cvtColor(goal, goal_hsv, cv::COLOR_BGR2HSV);

	//Color filtering masks matrices, leaves the HSV values NOT THE DETECTED COLOR
        cv::Mat mask_1, mask_2;
        cv::Mat mask_goal;
	// push for tav
	//yellow
        cv::Scalar goal_lower_bound = cv::Scalar(28, 80, 120);
        cv::Scalar goal_upper_bound = cv::Scalar(36, 255, 255);
	// orange
        //cv::Scalar goal_lower_bound = cv::Scalar(1, 120, 50);
        //cv::Scalar goal_upper_bound = cv::Scalar(12, 255, 255);	
	//red
	//cv::Scalar goal_lower_bound = cv::Scalar(120,50,50); //0 80 150
	//cv::Scalar goal_upper_bound = cv::Scalar(145,255,255);  // 15 255 255

	//green
        // cv::Scalar lower_bound_1 = cv::Scalar(41, 80, 80);
        // cv::Scalar upper_bound_1 = cv::Scalar(56, 255, 255);

	//purple
       cv::Scalar lower_bound_2 = cv::Scalar(115, 40, 30);
       cv::Scalar upper_bound_2 = cv::Scalar(150, 255, 255);

        cv::inRange(hsv_frame, goal_lower_bound, goal_upper_bound, mask_goal);

        //cv::inRange(hsv_frame, lower_bound_1, upper_bound_1, mask_1);
        cv::inRange(hsv_frame, lower_bound_2, upper_bound_2, mask_2);

	//cv::Mat masked_frame;
	//cv::bitwise_and(frame,frame,masked_frame,mask_2);

	// Balloon Detection
        if ((cam_mode == true) && (net_flag == false))
        {
		goal_flag = false;
		//RCLCPP_INFO(this->get_logger(), "balloon on bitch");
		//green taken out add contour 1 below
                std::vector<std::vector<cv::Point>>  contours_2, all_contours;
               // cv::findContours(mask_1, contours_1, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(mask_2, contours_2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                //all_contours.insert(all_contours.end(), contours_1.begin(), contours_1.end());
                all_contours.insert(all_contours.end(), contours_2.begin(), contours_2.end());

                cv::RotatedRect largest_contour;
                double largest_contour_area = 0;

		// Finding the contour with the largest area
               for (const auto &contour : all_contours) {
                   double contour_area = cv::contourArea(contour);
                   if (contour_area > largest_contour_area) {
                       largest_contour = cv::minAreaRect(contour);
                       largest_contour_area = contour_area;
                   }
               }
		// if the largest contour has width and has height, so it exists, then we find the radius and we append the points to a Point list so long as the are greater than the minimum radius and less than the maximum radius
                if (largest_contour.size.width != 0 && largest_contour.size.height != 0) {
                    cv::Point2f center = largest_contour.center;
                    int radius = std::max(largest_contour.size.width, largest_contour.size.height) / 2;
                    if (radius >= minimum_radius && radius <= maximum_radius) {
                        if (center.x >= 0 && center.x < frame.cols && center.y >= 0 && center.y < frame.rows) {
                            detected_coords.push_back(center);
                            // Draw a circle around the detected coordinate
                           cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 2);
                            // Print the detected coordinates
                          // std::cout << "Detected X: " << center.x << ", Detected Y: " << center.y << std::endl;
                        }
                    }
                }

                    // Calculate and print average coordinates
                    total_x = 0;
                    total_y = 0;
                    for (const auto &coord : detected_coords) {
                       total_x += coord.x;
                        total_y += coord.y;
                    }
                    if (!detected_coords.empty()) {
                       // RCLCPP_INFO(this->get_logger(),  "Average X: " << total_x / detected_coords.size() << ", Average Y: " << total_y / detected_coords.size());
			auto msg = blimp_interfaces::msg::CameraCoord(); // blimp interfaces message
                        avg_x = std::round(total_x/detected_coords.size()); // averaging the x we found to reduce noise
                        avg_y = std::round(total_y/detected_coords.size()); // averaging the y we found to reduce noise
                        msg.position = {avg_x,avg_y}; // setting the value of the ROS topic message
                        cam_data_publisher_->publish(msg); //publishing the message
                        //RCLCPP_INFO(this->get_logger(), "coords} avg_x: %i, avg_y: %i", avg_x, avg_y);
                        RCLCPP_INFO(this->get_logger(), "Publishing goal_flag: %d", goal_flag);
                        blimp_interfaces::msg::Bool flag_msg;
                        flag_msg.flag = goal_flag;  // Or set based on other conditions
                        cam_flag_publisher_->publish(flag_msg);
                    }
                    // Clear the detected coordinates for the next 10 frames
                    detected_coords.clear(); // clears the Point list in order to remove oversaturating the averaging values (too many means we will slowly centralize to a fixed point rather than the centers we need)

                //cv::imshow("Detected Color", frame);

        }

        else
        { 
	goal_flag = true;
	//RCLCPP_INFO(this->get_logger(),"Goal on biotch");
	// matrix for edge detection
          cv::Mat edges;
	// edge detection method provided from OpenCV, takes the frame(must be made grayscale by this point(1 channel)), an output matrix, and then 2 bounding threshold values
          cv::Canny(mask_goal, edges, low_threshold, high_threshold);
	// List to store the 4 values in Lines(x1, y1, x2, y2)
         std::vector<cv::Vec4i> linesP;
        // OpenCV method find lines between two points found in the edge detection matrix
	  cv::HoughLinesP(edges, linesP, rho, theta, threshold, min_line_length, max_line_gap);
          std::vector<cv::Point> midpoints;
          if (!linesP.empty()) {
              for (size_t i = 0; i < linesP.size(); i++) {
                  cv::Vec4i line = linesP[i];
                  int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
                  //finding midpoints
		  int mid_x = (x1 + x2) / 2;
                  int mid_y = (y1 + y2) / 2;
  		  //appending to a list for future computations
                  midpoints.push_back(cv::Point(mid_x, mid_y));
  
                  //cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
              }
  
              if (!midpoints.empty()) {
                 //RCLCPP_INFO(this->get_logger(), "DEsolation."); 
                // finds the max and min for both x and y
		// we use this to find the furthest out you can get from the center of the square
		// the min and max x tell us the vertical sides(left and right) of the square(because y wouldn't change in those instance)
		//the same goes for y and the horizontal(top and bottom) sides of the square.
		max_x = INT_MIN, min_x = INT_MAX, max_y = INT_MIN, min_y = INT_MAX;
  
                  for (const auto &point : midpoints) {
                      max_x = std::max(max_x, point.x);
                      min_x = std::min(min_x, point.x);
                      max_y = std::max(max_y, point.y);
                      min_y = std::min(min_y, point.y);
	//	      std::cout << "Detected Goal - Max X: " << max_x << ", Min X: " << min_x << ", Max Y: " << max_y << ", Min Y: "<< min_y << std::endl;

		      //RCLCPP_INFO(this->get_logger(), "min x: %i max x: %i min y: %i max y: %i", min_x, max_x, min_y, max_y);
                  }
  		 //finding the midpoint of the midpoints in order to find the center
		  //RCLCPP_INFO(this->get_logger(), "min x: %i max x: %i min y: %i max y: %i", min_x, max_x, min_y, max_y);
                  center_x = (min_x + max_x) / 2;
                  center_y = (min_y + max_y) / 2;
                  total_lines++;
                 // std::cout << "min: " << min_x << "max: " << max_x << std::endl;  
                 // cv::circle(frame, cv::Point(max_x, max_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(max_x, min_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(min_x, max_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(min_x, min_y), 5, cv::Scalar(0, 0, 255), -1);
                 // cv::circle(frame, cv::Point(center_x, center_y), 5, cv::Scalar(255, 0 , 0), -1);
  
  
               }
                  // publishing the message to blimp interfaces
		 //RCLCPP_INFO(this->get_logger(), "min x: %i max x: %i min y: %i max y: %i", min_x, max_x, min_y, max_y);
		  auto msg = blimp_interfaces::msg::CameraCoord();
                  msg.position = {center_x, center_y};
                  cam_data_publisher_->publish(msg);
                  RCLCPP_INFO(this->get_logger(), "Publishing goal_flag: %d", goal_flag);
                  blimp_interfaces::msg::Bool flag_msg;
                  flag_msg.flag = goal_flag;  // Or set based on other conditions
                  cam_flag_publisher_->publish(flag_msg);

    //RCLCPP_INFO(this->get_logger(), "max: %i min: %i", max_x, min_y);
		
              midpoints.clear(); // removing oversaturation of averaging data
              //std::cout << "Goal - X: " << center_x << ", Y: " << center_y << std::endl;
          }
       }

    
         //cv::imshow("Detection", masked_frame);

         if(cv::waitKey(1) == 27){
           return;
         }

    
    }


    // members 
    const int minimum_radius = 5; //pixels
    const int maximum_radius = 300; //pixels
    bool cam_mode;
    bool goal_flag;
    bool net_flag;
    int avg_x;
    int avg_y;
    int frame_count = 0;
    std::vector<cv::Point2f> detected_coords;
    float total_x = 0;
    float total_y = 0;

    rclcpp::Publisher<blimp_interfaces::msg::CameraCoord>::SharedPtr cam_data_publisher_;
    rclcpp::Publisher<blimp_interfaces::msg::Bool>::SharedPtr cam_flag_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_;
    rclcpp::Subscription<blimp_interfaces::msg::Bool>::SharedPtr subscriber_net_servo;

    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t frame_count_;
    int minimum_radius_;
    int center_x;
    int center_y;
    float rho;
    float theta;
    int threshold;
    int min_line_length;
    int max_line_gap;
    int low_threshold;
    int high_threshold;
    int total_lines;
    int x_button;
    int max_x, min_x, min_y, max_y;
    bool findGoal;
};

// Necessary ROS node functions
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    node->cap_.release();
    cv::destroyAllWindows();
    return 0;
}
