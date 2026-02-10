#include "rclcpp/rclcpp.hpp"
#include "blimp_interfaces/msg/cart_coord.hpp"
#include "blimp_interfaces/msg/esc_input.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <sstream>
#include "blimp_interfaces/msg/camera_coord.hpp"

class BalloonEscInput : public rclcpp::Node // MODIFY NAME
{
public:
    BalloonEscInput() : Node("balloon_esc_input"), // MODIFY NAME
    coord_{640, 360} {   // Force Coefficients for each motor 
        // includes the direction of thrust (x,y,z) that each thruster produces
        // includes the moments arms between the thruster and center of gravity 
        // 1 = left motor, 2 = right motor, 3 = up motor (left-so negative), 4 = down motor

        // Fx1 = 1, Fx2 = 1, Fx3 = 0, Fx4 = 0;
        // Fy1 = 0, Fy2 = 0, Fy3 = 0, Fy4 = 0;
        // Fz1 = 0, Fz2 = 0, Fz3 = -1, Fz4 = 1;
        // lx1 = 0.04, lx2 = 0.04, lx3 = 0.07, lx4 = 0.07;
        // ly1 = -0.51, ly2 = 0.51, ly3 = -0.09, ly4 = 0.09;
        // lz1 = -0.26, lz2 = -0.26, lz3 = 0.26, lz4 = 0.26;

        // Thrust Coefficient Matrix (values comes form thrust stand testing)
        // Approximated as a linear function so Thrust [N] = K * PWM + B
        // K matrix is the slope of the function relating Thrust and PWM input
        //K = Eigen::DiagonalMatrix<double, 4>(0.000602, 0.000602, 0.00153807, 0.00153807);
        // B vector represents the Y-intercept of the linear function 
        // B << -0.632456,
        //      -0.632456,
        //      -1.75866439,
        //      -1.75866439;
        // [6 x 4] Matrix for all 4 thrusters. 
        // Column 1 = forces and moments for left thruster
        // Column 2 = forces and moments for right thruster
        // Column 3 = "..." up thruster
        // Column 4 = "..." down thruster
        // Q <<   Fx1,             Fx2,             Fx3,             Fx4, 
        //        Fy1,             Fy2,             Fy3,             Fy4, 
        //        Fz1,             Fz2,             Fz3,             Fz4, 
        //        Fz1*ly1-Fy1*lz1, Fz2*ly2-Fy2*lz2, Fz3*ly3-Fy3*lz3, Fz4*ly4-Fy4*lz4,
        //        Fx1*lz1-Fz1*lx1, Fx2*lz2-Fz2*lx2, Fx3*lz3-Fz3*lx3, Fx4*lz4-Fz4*lx4,
        //        Fy1*lx1-Fx1*ly1, Fy2*lx2-Fx2*ly2, Fy3*lx3-Fx3*ly3, Fy4*lx4-Fx4*ly4;
        //Since Force Coefficent Matrix is not square we need to make it square to be able to take the inverse
        // Q++ is the Moorse-psuedo inverse of Q
        // T = Q.transpose()*Q;
        // Qp = T.inverse()*Q.transpose();
        // Taking Inverse of Thrust Coefficents
        // Wanting to solve tau = Q*K*u, for u
        // K_inv = K.inverse();

        this->declare_parameter<int>("x_goal", 640);
        this->declare_parameter<double>("y_goal", 360);
        coord_ = {0, 0};

        x_goal_ = this->get_parameter("x_goal").as_int();
        y_goal_ = this->get_parameter("y_goal").as_double();

        // CREATING A SUBSCRIBER TO THE INV_KINE NODE. CartCoord is the interface type forces is the topic and the bind is the callback we need to send the info too
        subscriber_ = this->create_subscription<blimp_interfaces::msg::CartCoord>(
            "forces", 10, std::bind(&BalloonEscInput::callback_force_to_esc, this, std::placeholders::_1));

        // Subscribing to Camera
        subscriber_camera = this->create_subscription<blimp_interfaces::msg::CameraCoord>(
            "cam_data", 3, std::bind(&BalloonEscInput::callback_cam_data, this, std::placeholders::_1));

        // publisher for the ESCinputs
        publisher_ = this->create_publisher<blimp_interfaces::msg::EscInput>("ESC_balloon_input", 10);
        // timer for the main callback
        timer_ = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&BalloonEscInput::callback_timer, this));
        RCLCPP_INFO(this->get_logger(), "Forces are converted");
        tau.setZero();
    }

private:
    void callback_cam_data(const blimp_interfaces::msg::CameraCoord::SharedPtr msg) {
        //retriving camera data from the /cam_data topic and storing as a global variable
        coord_[0] = msg->position[0];
        coord_[1] = msg->position[1];
        // haveing a timer start every time this callback is used   
    }


    void callback_force_to_esc(const blimp_interfaces::msg::CartCoord::SharedPtr msg) {
        // reading the values of the msg in to a 1 x 6 force vector
        tau << msg->x+0.00,
                  msg->y,
                  msg->z,
                  msg->theta,
                  msg->phi,
                  msg->psy;

        x_error = x_goal_ - coord_[0];
        y_error = coord_[1] -  y_goal_;


        Fy1 = 0, Fy2 = 0, Fy3 = 0;
        //if (tau(2) > 0) {
        if (y_error > 0) {
            Fz1 = 0, Fz2 = 0, Fz3 = 1;
            k3 = 0.001112;
            b3 = -1.748142;
        } else {
            Fz1 = 0, Fz2 = 0, Fz3 = -1;
            k3 = -0.000554;
            b3 = 0.788902;
        }

        if (x_error > 0) {
            Fx1 = 1, Fx2 = -1, Fx3 = 0;
            k1 = 0.001112;
            b1 = -1.748142;
            k2 = -0.000554;
            b2 = 0.788902;
        } else {
            Fx1 = -1, Fx2 = 1, Fx3 = 0;
            k2 = 0.001112;
            b2 = -1.748142;
            k1 = -0.000554;
            b1 = 0.788902;
        }

        // 400 grams:
        lx1 = 0.0, lx2 = 0.0, lx3 = 0.0;
        ly1 = -0.461, ly2 = 0.461, ly3 = 0.0;
        lz1 = -0.175, lz2 = -0.175, lz3 = 0.178;

        // 600 grams: 
        //lx1 = 0.0, lx2 = 0.0, lx3 = 0.0;
        //ly1 = -0.506, ly2 = 0.506, ly3 = 0.0;
        //lz1 = -0.258, lz2 = -0.258, lz3 = 0.257;

        K = Eigen::DiagonalMatrix<double, 3>(k1, k2, k3);

        B << b1, 
             b2, 
             b3;

        Q <<    Fx1,             Fx2,             Fx3,              
                Fy1,             Fy2,             Fy3,              
                Fz1,             Fz2,             Fz3,            
                Fz1*ly1-Fy1*lz1, Fz2*ly2-Fy2*lz2, Fz3*ly3-Fy3*lz3,
                Fx1*lz1-Fz1*lx1, Fx2*lz2-Fz2*lx2, Fx3*lz3-Fz3*lx3,
                Fy1*lx1-Fx1*ly1, Fy2*lx2-Fx2*ly2, Fy3*lx3-Fx3*ly3;

        T = Q.transpose()*Q;
        Qp = T.inverse()*Q.transpose();
        // Taking Inverse of Thrust Coefficents
        // Wanting to solve tau = Q*K*u, for u
        K_inv = K.inverse();
    }

    void callback_timer() {
        // getting the speudo inverse for the forces to conver to motor inputs
        Eigen::Matrix<double, 3,1> F = K_inv*Qp*tau - K_inv*B;

        //assighning the msg and values to the msg then publishing it
        auto msg2 = blimp_interfaces::msg::EscInput();
        msg2.esc_pins = {5,6,13};
        msg2.pwm_l = F(1,0); 
        msg2.pwm_r = F(0,0);
        //msg2.pwm_u = F(2,0); //are these correct
        msg2.pwm_d = F(2,0);
        RCLCPP_INFO(this->get_logger(), "M1: %f  M2: %f  M3: %f", msg2.pwm_l, msg2.pwm_r, msg2.pwm_d);
        publisher_->publish(msg2);
        //RCLCPP_INFO(this->get_logger(), "M1: %f  M2: %f  M3: %f  M4: %f", F(0), F(1), F(2), F(3));

    }


    // Members of the Class

    float Fx1, Fx2, Fx3, Fx4;
    float Fy1, Fy2, Fy3, Fy4;
    float Fz1, Fz2, Fz3, Fz4;
    float lx1, lx2, lx3, lx4;
    float ly1, ly2, ly3, ly4;
    float lz1, lz2, lz3, lz4;
    double f1, f2, f3, f4;
    float k1, k2, k3;
    float b1, b2, b3;
    int x_error, y_error;
    int x_goal_;
    double y_goal_;
    std::vector<int> coord_;
    Eigen::DiagonalMatrix<double, 3> K;
    Eigen::DiagonalMatrix<double, 3> K_inv;
    Eigen::Matrix<double, 6,1> tau;
    Eigen::Matrix<double, 6,3> Q;
    Eigen::Matrix<double, 3,3> T;
    Eigen::Matrix<double, 3,6> Qp;
    Eigen::Matrix<double, 3,1> F;
    Eigen::Matrix<double, 3,1> B;
    
    // pointers for ros2
    rclcpp::Subscription<blimp_interfaces::msg::CartCoord>::SharedPtr subscriber_;
    rclcpp::Subscription<blimp_interfaces::msg::CameraCoord>::SharedPtr subscriber_camera;
    rclcpp::Publisher<blimp_interfaces::msg::EscInput>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BalloonEscInput>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
