#include "rclcpp/rclcpp.hpp" //ros2 package for C++
#include "blimp_interfaces/msg/imu_data.hpp" //custom variable type message for IMU data (float32)
#include "blimp_interfaces/msg/cart_coord.hpp" //custom variable type message for cartesian coordinates (float64)
#include "blimp_interfaces/msg/baro_data.hpp" //custom variable type message for barometer data (float64)
#include "blimp_interfaces/msg/esc_input.hpp" //custom variable type message for ESC input (float64)
#include <Eigen/Dense> //library for doing matric math in C++
#include <cmath> //library for additionl math functionality in C++
#include <string> // library to inlcude string data type in C++

using namespace std::chrono_literals; //gain access to the classes in the chrono library, the corresponding literal operators become visible as well.

// Creating a dynamic model class and Node using rclcpp package for C+
class DynamicModel : public rclcpp::Node {
public: // Defing public class (can be accesed outside of scope)
    DynamicModel() //constructor for dynamic model class
    : Node("dynamic_model"), // ros2 node name
    height{0.0}, 
    m{0.460386}, // mass [kg] // 400 grams: 0.6039 // 600 grams: 0.72838 
    zg{0.178263} // z_directional center of gravity [m] // 400 grams: // 600 grams: 0.26738
    {
        this->declare_parameter<double>("rho_air", 1.225); // declaring air density parameter as type double with default value 1.225 [kg/m^3]
        // 400 grams:
        this->declare_parameter<double>("buoyancy", 0.460386*9.81); // declaring blimp bouyanvy parameter as type double with default parameter equal weight of blimp [N]
        // 600 grams:
        // this->declare_parameter<double>("buoyancy", 0.72838*9.81);

        rho_air = this->get_parameter("rho_air").as_double(); // creating air density variable with name "rho_air"
        B = this->get_parameter("buoyancy").as_double(); // creating buyancy variable with name "B" 
        W = m * 9.81; // creating Weight variable "W"
        //        a     b      density    Ix         Iy        Iz
        // 400 grams:
        M_matrix(0.9068,0.37233,rho_air,0.046568,0.087299,0.073692); // Initilizing M_matrix (Mass Matrix)
        // 600 grams:
        // M_matrix(1,0.4098,rho_air,0.08034,0.13571,0.10387); // Initilizing M_matrix (Mass Matrix)

        D_matrix(); // Initilizing D_matrix (Damping Matrix)

        R_imu << 1, 0, 0, // Rotation matrix to get IMU into body cody coordinate system
                  0,-1, 0, // 180 degree roation about x-axis
                  0, 0,-1;

        // subscribing to "imu_data" topic, with custom data type form blimp interface. Reference to class Dynamic model with callback to function
        subscriber_imu = this->create_subscription<blimp_interfaces::msg::ImuData>(
            "imu_data", 10, std::bind(&DynamicModel::callback_imu, this, std::placeholders::_1));
        // subcribing to "barometer_data" topic "..."
        subscriber_baro = this->create_subscription<blimp_interfaces::msg::BaroData>(
            "barometer_data", 10, std::bind(&DynamicModel::callback_baro, this, std::placeholders::_1));
        // subscribing to "balloon_inpu"t topic "..."
        subscriber_dynamic_model = this->create_subscription<blimp_interfaces::msg::CartCoord>(
            "balloon_input", 10, std::bind(&DynamicModel::callback_dynamic_model, this, std::placeholders::_1));
        // subscribing to "forces" topic
        kine_publisher = this->create_publisher<blimp_interfaces::msg::CartCoord>("forces", 10);
        RCLCPP_INFO(this->get_logger(), "Dynamics are modeled"); //ros2 print telling user the Dynamics are modled

        // ... (initialize matrices here as needed) ...
        time(&start); // Initialalizing time
    }
    // initlizing variable types for parameters used in mass matrix (type double)
    void M_matrix(double a, double b, double rho, double Ix, double Iy, double Iz) {
        // eccentricity of Blimp:
        double e = std::sqrt((1 - (b/a) * (b/a)));
        // Lamda K-factor constants for finding Added mass Matrix values:
        double beta0 = (1 / (e * e)) - ((std::log((1 + e) / (1 - e)) * ((1 - e * e) / (2 * e * e * e))));
        double alpha0 = ((2 * (1 - e * e)) / (e * e * e)) * ((0.5 * std::log((1 + e) / (1 - e))) - e);
        double kprime = (e * e * e * e * (beta0 - alpha0)) / ((2 - e * e) * (2 * e * e - (2 - e * e) * (beta0 - alpha0)));
        double k1 = alpha0 / (2 - alpha0);
        double k2 = beta0 / (2 - beta0);
        double Izh = (4/5) * M_PI * rho * a * b * b * (a * a + b * b);
        mx_prime = m + k1 * m;
        my_prime = m + k2 * m;
        mz_prime = m + k2 * m;
        Ix_prime_ = Ix;
        Iy_prime_ = Iy + kprime * Izh;
        Iz_prime_ = Iz + kprime * Izh;
        
        // Mass matrix composed of the effects of rigid body mass (moments of inertia and masses)
        // Mass matrix also accounts for the effects of virtual or added mass (how the fluid around the blimp affects the motion)
        M << mx_prime, 0, 0, 0, m * zg, 0,
        0, my_prime, 0, -m * zg, 0, 0,
        0, 0, mz_prime, 0, 0, 0,
        0, -m * zg, 0, Ix_prime_, 0, 0,
        m * zg, 0, 0, 0, Iy_prime_, 0,
        0, 0, 0, 0, 0, Iz_prime_;
    }
        // Damping matrix which accounts for the viscous effects on the blimp
    void D_matrix() {
        D = Eigen::DiagonalMatrix<double, 6>(0.022, 0.1745, 0.21, .02, 0.04, 0.04); // constant vlaues used here found from vicon testing
    }

private:
    // ... (function definitions to compute M and D matrices) ...
    // C++ barometer function "callback_baro" which was declared above when subscribing to barometer data
    void callback_baro(const blimp_interfaces::msg::BaroData::SharedPtr msg) {
        height = msg->height; // height from barometer message
        time(&finish); // ending timer
        dt = difftime(finish, start); // difference between start time and finsih time used to approximate the vertical velocity form the barometer
        time(&start); // initlzing start time again so we can calulcate change in time after each run through

         if (dt > 0) { // Prevent division by zero.
             vz1 = ((height_old - height) / dt)*-1; // vertical velicty equals change in height from barometer dividing by change in time
             height_old = height; // setting the old height used in the function equal to height at current time step
         
             // Add new value to the buffer
             // Moving average for vertical velocity
             if (vz_buffer.size() >= AVG_WINDOW) {
                 // If the buffer is full, remove the oldest value
                 std::rotate(vz_buffer.begin(), vz_buffer.begin() + 1, vz_buffer.end());
                 vz_buffer.back() = vz1;
             } else {
                 // If the buffer is not full, just add the new value
                 vz_buffer.push_back(vz1);
             }
         
            // Calculate the moving average of vz
            sum = std::accumulate(vz_buffer.begin(), vz_buffer.end(), 0.0);
            vz = sum / vz_buffer.size();
         }
    }
    // calling function callback_imu to get data from imu topic
    void callback_imu(const blimp_interfaces::msg::ImuData::SharedPtr msg) {
        // linear acceleration [m/s]: [0] = ax,, [1] = ay, [2] = az 
        lin_accel << msg->imu_lin_accel[0], msg->imu_lin_accel[1], msg->imu_lin_accel[2];
        // Euler angles [rad]: [0] = roll(phi), [1] = pitch(theta), [2] = yaw[psi]
        euler << msg->imu_euler[0], msg->imu_euler[1], msg->imu_euler[2];
        // angular velocity [rad/s]: [0] = phi_dot, [1] = theta_dot, [2] = psi_dot
        gyro << msg->imu_gyro[0], msg->imu_gyro[1], msg->imu_gyro[2];

        // using rotation matrix to convert euler angles to body frame
        lin_accel = R_imu * lin_accel;
        euler = R_imu * euler;
        gyro = R_imu * gyro;        
    }
    // calling function "callback_dynamic_model"
    void callback_dynamic_model(const blimp_interfaces::msg::CartCoord::SharedPtr msg) {
        
        //RCLCPP_INFO(this->get_logger(), "time: %f  height: %f  height_old: %f  vz: %f", dt, height, height_old,vz);
        vx = 0.0; // Estimating forward velocity (need some state estimation to determine this)
        vy = 0.0; // Estimating side-to-side velocity
        vel << vx, vy, vz, gyro(0), gyro(1), gyro(2); // creating [6x1] velocity vector "vel" with linar and angular veloicty
        

        accel << msg->x,msg->y,msg->z,msg->theta,msg->phi,msg->psy; // creating acceleration vector [6x1] using msg which comes from PI controller

        // Coriolis matrix
        C << 0, 0, 0, 0, -(mz_prime * vz) - (my_prime * vy - m * zg * gyro(0)), 0,
            0, 0, 0, -(mz_prime * vz), 0, -(mx_prime * vx - m * zg * gyro(1)),
            0, 0, 0, -(my_prime * vy - m * zg * gyro(0)), -(mx_prime * vx + m * zg * gyro(1)), 0,
            0, -(mz_prime * vz), -(my_prime * vy - m * zg * gyro(0)), 0, -(Iz_prime_ * gyro(2)), -(m * zg * vx + Iy_prime_ * gyro(1)),
            -(mz_prime * vz), 0, -(mx_prime * vx - m * zg * gyro(1)), -(Iz_prime_ * gyro(2)), 0, -(m * zg * vy - Ix_prime_ * gyro(0)),
            -(my_prime * vy - m * zg * gyro(0)), -(mx_prime * vx + m * zg * gyro(1)), 0, -(m * zg * vx - Iy_prime_ * gyro(1)), -(m * zg * vy + Ix_prime_ * gyro(0)), 0;
        
        // Restoring Forces matrix
        g << (W - B) * sin(euler(1)),
            -(W - B) * cos(euler(1)) * sin(euler(0)),
            -(W - B) * cos(euler(1)) * cos(euler(0)),
            zg * W * cos(euler(1)) * sin(euler(0)),
            zg * W * sin(euler(1)),
            0;       
        // Propulsive forces and moments = tau[6x1]
        // Solving this vector using kinematic model
        tau = M*accel + C*vel + g + D*vel; // + D*vel + C*vel + g;

        auto msg2 = blimp_interfaces::msg::CartCoord(); // creating variable msg2 with data structure from custom blimp interfaces
        msg2.x = tau(0); // force in x-direction
        msg2.y = tau(1); // force in y-direction
        msg2.z = tau(2); // force in z-direction
        msg2.theta = tau(3); // moment about x-axis
        msg2.phi = tau(4); // moment about y-axis
        msg2.psy = tau(5); // // moment about z-axis
        kine_publisher->publish(msg2);
        RCLCPP_INFO(this->get_logger(), "taux: %f  tauy: %f  tauz: %f  tauth: %f  tauxphi: %f  tauxpsy: %f", tau(0), tau(1), tau(2), tau(3), tau(4), tau(5));
        //RCLCPP_INFO(this->get_logger(), "taux: %f  height: %f  height_old: %f", tau(0), height, height_old);
        //RCLCPP_INFO(this->get_logger(), "euler1: %f  euler2: %f  euler3: %f", euler(0), euler(1), euler(2));
        //RCLCPP_INFO(this->get_logger(), "gyro1 : %f  gyro 2: %f  gyro 3: %f", gyro(0), gyro(1), gyro(2));
        //RCLCPP_INFO(this->get_logger(), "Start time: %ld", start);
    }



    // Members for the node
    // Constant for AVG_WINDOW length
    static constexpr size_t AVG_WINDOW = 5;
    
    // Buffer to store history of vz values
    std::vector<double> vz_buffer; // double vector with name vz_buffer
    double height; // double variable named height
    double m; // double variable name m (mass)
    double zg; // double variable named zg (z-directional center of gravity)
    double rho_air; // double variable name rho_air (air density)
    double dt; // double variable name dt (change in time)
    double B; // double varaible named B (Buoyancy)
    double W; // double varaible name W (Weight)
    double mx_prime, my_prime, mz_prime; // double variables mx_prime, my_prime, mz_prime (used in mass matrix)
    double Ix_prime_, Iy_prime_, Iz_prime_; // double varaibales Ix_prime, Iy_prime, Iz_prime (used in mass matrix)
    // Uses Eigen library
    Eigen::DiagonalMatrix<double, 6> D; // 6x6 diagonal matrix (Damping matrix of type double)
    Eigen::Matrix<double, 6, 6> M; // 6x6 matrix (Mass matrix of type double)
    Eigen::Matrix3d R_imu; // Replace the rotation matrix defined in Python
    time_t start, finish; //
    double vx;
    double vy;
    double vz;
    double vz1;
    double height_old;
    double sum;
    Eigen::Matrix<double, 6, 1> tau;
    Eigen::Matrix<double, 6, 1> g;
    Eigen::Matrix<double, 6, 6> C;
    Eigen::Vector3d gyro;
    Eigen::Matrix<double, 6,1> vel;
    Eigen::Matrix<double, 6,1> accel;
    Eigen::Vector3d lin_accel;
    Eigen::Vector3d euler;
    rclcpp::Subscription<blimp_interfaces::msg::ImuData>::SharedPtr subscriber_imu;
    rclcpp::Subscription<blimp_interfaces::msg::BaroData>::SharedPtr subscriber_baro;
    rclcpp::Subscription<blimp_interfaces::msg::CartCoord>::SharedPtr subscriber_dynamic_model;
    rclcpp::Publisher<blimp_interfaces::msg::CartCoord>::SharedPtr kine_publisher;
    // ... (other member variables and matrices) ...
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicModel>());
    rclcpp::shutdown();
    // Use 'std::system' if you really need to call "sudo killall pigpiod", but consider the security implications
    return 0;
}
