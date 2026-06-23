#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_controller/action/arm_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <cmath>
#include <regex>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using ArmInterface = arm_controller::action::ArmInterface;
using GoalHandleArmInterface = rclcpp_action::ServerGoalHandle<ArmInterface>;

class Kinematics {
public:
    Kinematics() : L0(0.0), L1(0.0), L2(0.0), L3(0.0) {
        servo_angle.resize(4, 0.0);
        servo_pwm.resize(4, 0);
    }

    void setup_kinematics(double L0 = 100, double L1 = 105, double L2 = 88, double L3 = 155) {
        this->L0 = L0 * 10;
        this->L1 = L1 * 10;
        this->L2 = L2 * 10;
        this->L3 = L3 * 10;
    }

    int kinematics_analysis(double x, double y, double z, double Alpha) {
        const double pi = M_PI;
        double x_scaled = x * 10;
        double y_scaled = y * 10;
        double z_scaled = z * 10;

        double l0 = L0;
        double l1 = L1;
        double l2 = L2;
        double l3 = L3;

        double theta6;
        if (x_scaled == 0) {
            theta6 = 0.0;
        } else {
            theta6 = atan2(x_scaled, y_scaled) * 270.0 / pi;
        }

        y_scaled = sqrt(x_scaled * x_scaled + y_scaled * y_scaled);
        y_scaled = y_scaled - l3 * cos(Alpha * pi / 180.0);
        z_scaled = z_scaled - l0 - l3 * sin(Alpha * pi / 180.0);

        if (z_scaled < -l0) return 1;
        if (sqrt(y_scaled * y_scaled + z_scaled * z_scaled) > (l1 + l2)) return 2;

        double ccc = acos(y_scaled / sqrt(y_scaled * y_scaled + z_scaled * z_scaled));
        double bbb = (y_scaled * y_scaled + z_scaled * z_scaled + l1 * l1 - l2 * l2) / 
                     (2 * l1 * sqrt(y_scaled * y_scaled + z_scaled * z_scaled));

        if (bbb > 1 || bbb < -1) return 5;

        int zf_flag = (z_scaled < 0) ? -1 : 1;
        double theta5 = ccc * zf_flag + acos(bbb);
        theta5 = theta5 * 180.0 / pi;

        if (theta5 > 180.0 || theta5 < 0.0) return 6;

        double aaa = -(y_scaled * y_scaled + z_scaled * z_scaled - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        if (aaa > 1 || aaa < -1) return 3;

        double theta4 = acos(aaa);
        theta4 = 180.0 - theta4 * 180.0 / pi;

        if (theta4 > 135.0 || theta4 < -135.0) return 4;

        double theta3 = Alpha - theta5 + theta4;

        if (theta3 > 90.0 || theta3 < -90.0) return 7;

        servo_angle[0] = theta6;
        servo_angle[1] = theta5 - 90;
        servo_angle[2] = theta4;
        servo_angle[3] = theta3;

        servo_pwm[0] = static_cast<int>(1500 - 2000.0 * servo_angle[0] / 270.0);
        servo_pwm[1] = static_cast<int>(1500 + 2000.0 * servo_angle[1] / 270.0);
        servo_pwm[2] = static_cast<int>(1500 + 2000.0 * servo_angle[2] / 270.0);
        servo_pwm[3] = static_cast<int>(1500 - 2000.0 * servo_angle[3] / 270.0);

        return 0;
    }

    std::string kinematics_move(double x, double y, double z, int time_ms) {
        if (y < 0) return "";

        int min_alpha = 0;
        int flag = 0;

        for (int i = 0; i >= -135; i--) {
            if (kinematics_analysis(x, y, z, i) == 0) {
                if (i < min_alpha) {
                    min_alpha = i;
                }
                flag = 1;
            }
        }

        if (flag) {
            kinematics_analysis(x, y, z, min_alpha);
            char cmd[256];
            snprintf(cmd, sizeof(cmd), 
                "{#000P%04dT%04d!#001P%04dT%04d!#002P%04dT%04d!#003P%04dT%04d!}",
                servo_pwm[0], time_ms,
                servo_pwm[1], time_ms,
                servo_pwm[2], time_ms,
                3000 - servo_pwm[3], time_ms);
            RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "Kinematics move - x: %.2f, y: %.2f, z: %.2f, L0: %.2f, L1: %.2f, L2: %.2f, L3: %.2f, Command: %s", 
                       x, y, z, L0, L1, L2, L3, cmd);
            return std::string(cmd);
        }

        return "";
    }

private:
    double L0, L1, L2, L3;
    std::vector<double> servo_angle;
    std::vector<int> servo_pwm;
};

class SerialController {
public:
    SerialController(const std::string& port_name = "/dev/ttyUSB0", int baudrate = 115200)
        : port_name_(port_name), baudrate_(baudrate), fd_(-1) {}

    ~SerialController() {
        close();
    }

    bool open() {
        fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialController"), "Failed to open serial port: %s", port_name_.c_str());
            return false;
        }

        struct termios options;
        tcgetattr(fd_, &options);

        cfsetispeed(&options, baudrate_);
        cfsetospeed(&options, baudrate_);

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;

        tcsetattr(fd_, TCSANOW, &options);

        int rts = TIOCM_RTS;
        ioctl(fd_, TIOCMBIC, &rts);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        tcflush(fd_, TCIFLUSH);
        tcflush(fd_, TCOFLUSH);

        RCLCPP_INFO(rclcpp::get_logger("SerialController"), "Serial port %s opened successfully", port_name_.c_str());
        return true;
    }

    bool send_command(const std::string& command) {
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialController"), "Serial port not open!");
            return false;
        }

        ssize_t bytes_written = ::write(fd_, command.c_str(), command.size());
        if (bytes_written != static_cast<ssize_t>(command.size())) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialController"), "Failed to send command");
            return false;
        }

        tcdrain(fd_);
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
            RCLCPP_INFO(rclcpp::get_logger("SerialController"), "Serial port closed");
        }
    }

    bool is_open() const {
        return fd_ >= 0;
    }

private:
    std::string port_name_;
    int baudrate_;
    int fd_;
};

double calculate_execution_time(const std::string& command) {
    std::regex pattern(R"(T(\d{4})!)");
    std::smatch matches;
    std::string::const_iterator search_start(command.cbegin());

    int total_ms = 0;
    int count = 0;

    while (std::regex_search(search_start, command.cend(), matches, pattern)) {
        total_ms += std::stoi(matches[1].str());
        count++;
        search_start = matches.suffix().first;
    }

    if (count == 0) {
        return 1.0;
    }

    total_ms += (count - 1) * 20;
    return total_ms / 1000.0;
}

bool set_servo(SerialController& serial_controller, int index, int pwm, int time_ms = 1500) {
    if (!serial_controller.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("set_servo"), "Serial port not initialized!");
        return false;
    }

    if (index < 0 || index > 5) {
        RCLCPP_ERROR(rclcpp::get_logger("set_servo"), "Servo index must be between 0-5, current: %d", index);
        return false;
    }

    if (pwm < 0 || pwm > 3000) {
        RCLCPP_ERROR(rclcpp::get_logger("set_servo"), "PWM value must be between 0-3000, current: %d", pwm);
        return false;
    }

    int pwm_value = (index == 3) ? 3000 - pwm : pwm;

    char command[32];
    snprintf(command, sizeof(command), "#%03dP%04dT%04d!", index, pwm_value, time_ms);

    RCLCPP_INFO(rclcpp::get_logger("set_servo"), "Setting servo %d: PWM=%d, time=%dms", index, pwm_value, time_ms);

    bool success = serial_controller.send_command(command);

    if (success) {
        double exec_time = time_ms / 1000.0 + 0.02;
        RCLCPP_INFO(rclcpp::get_logger("set_servo"), "Command sent, expected execution time: %.2f seconds", exec_time);
        std::this_thread::sleep_for(std::chrono::duration<double>(exec_time));
        RCLCPP_INFO(rclcpp::get_logger("set_servo"), "Servo %d set completed", index);
    }

    return success;
}

bool move_to(SerialController& serial_controller, Kinematics& kinematics, 
             double x, double y, double z, int time_ms = 2000) {
    if (!serial_controller.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("move_to"), "Serial port not initialized!");
        return false;
    }

    std::string command = kinematics.kinematics_move(x, y, z, time_ms);

    if (command.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("move_to"), "Error: No valid inverse kinematics solution found!");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("move_to"), "Sending command: %s", command.c_str());

    double exec_time = calculate_execution_time(command);

    if (!serial_controller.send_command(command)) {
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("move_to"), "Command sent, expected execution time: %.2f seconds", exec_time);
    RCLCPP_INFO(rclcpp::get_logger("move_to"), "Waiting for arm to reach target position...");

    std::this_thread::sleep_for(std::chrono::duration<double>(exec_time / 2 + 1.0));

    RCLCPP_INFO(rclcpp::get_logger("move_to"), "Movement completed!");
    return true;
}

class ArmManageServer : public rclcpp::Node
{
public:
    explicit ArmManageServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("arm_manage_server", options),
          serial_controller_("/dev/ttyUSB0", 115200),
          kinematics_(),
          latest_apple_x_(0.0),
          latest_apple_y_(0.0),
          latest_apple_z_(0.0),
          got_apple_pose_(false),
          calibration_loaded_(false)
    {
        using namespace std::placeholders;

        kinematics_.setup_kinematics(100, 105, 88, 155);

        load_calibration_result();

        RCLCPP_INFO(this->get_logger(), "Initializing serial port...");
        if (!serial_controller_.open()) {
            RCLCPP_WARN(this->get_logger(), "Failed to open serial port, arm control will be simulated");
        }

        apple_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/apple_pose", 10,
            std::bind(&ArmManageServer::apple_pose_callback, this, std::placeholders::_1)
        );

        this->action_server_ = rclcpp_action::create_server<ArmInterface>(
            this,
            "arm_interface",
            std::bind(&ArmManageServer::handle_goal, this, _1, _2),
            std::bind(&ArmManageServer::handle_cancel, this, _1),
            std::bind(&ArmManageServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Arm manage action server started. Listening on 'arm_interface'");
    }

    ~ArmManageServer() {
        serial_controller_.close();
    }

private:
    rclcpp_action::Server<ArmInterface>::SharedPtr action_server_;
    SerialController serial_controller_;
    Kinematics kinematics_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr apple_pose_subscriber_;
    
    double latest_apple_x_;
    double latest_apple_y_;
    double latest_apple_z_;
    bool got_apple_pose_;
    
    double R_[9];
    double t_[3];
    bool calibration_loaded_;

    void apple_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        latest_apple_x_ = msg->point.x;
        latest_apple_y_ = msg->point.y;
        latest_apple_z_ = msg->point.z;
        got_apple_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "Received apple pose: X=%.4f, Y=%.4f, Z=%.4f", 
                   latest_apple_x_, latest_apple_y_, latest_apple_z_);
    }

    void load_calibration_result()
    {
        std::string calibration_file = "/home/hl/qyu_ws/src/hand_eye_calibration/hand_eye_calibration/calibration_result.yaml";
        
        try {
            YAML::Node config = YAML::LoadFile(calibration_file);
            
            if (config["calibration"] && config["calibration"]["rotation_matrix"]) {
                std::vector<double> rot_matrix = config["calibration"]["rotation_matrix"].as<std::vector<double>>();
                if (rot_matrix.size() >= 9) {
                    for (int i = 0; i < 9; ++i) {
                        R_[i] = rot_matrix[i];
                    }
                }
            }
            
            if (config["calibration"] && config["calibration"]["translation_vector"]) {
                std::vector<double> trans_vector = config["calibration"]["translation_vector"].as<std::vector<double>>();
                if (trans_vector.size() >= 3) {
                    for (int i = 0; i < 3; ++i) {
                        t_[i] = trans_vector[i];
                    }
                }
            }
            
            calibration_loaded_ = true;
            RCLCPP_INFO(this->get_logger(), "Calibration result loaded successfully");
        } catch (const YAML::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration result: %s", e.what());
            calibration_loaded_ = false;
        }
    }

    bool transform_camera_to_arm(double camera_x, double camera_y, double camera_z,
                                   double& arm_x, double& arm_y, double& arm_z)
    {
        if (!calibration_loaded_) {
            RCLCPP_WARN(this->get_logger(), "Calibration not loaded, using camera coordinates directly");
            arm_x = camera_x * 1000;
            arm_y = camera_y * 1000;
            arm_z = camera_z * 1000;
            return true;
        }

        arm_x = R_[0] * camera_x + R_[1] * camera_y + R_[2] * camera_z + t_[0];
        arm_y = R_[3] * camera_x + R_[4] * camera_y + R_[5] * camera_z + t_[1];
        arm_z = R_[6] * camera_x + R_[7] * camera_y + R_[8] * camera_z + t_[2];

        RCLCPP_INFO(this->get_logger(), "Camera to Arm transformation: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)",
                   camera_x, camera_y, camera_z, arm_x, arm_y, arm_z);
        
        return true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ArmInterface::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with command: %s", goal->command.c_str());
        
        (void)uuid;
        
        if (is_valid_command(goal->command)) {
            RCLCPP_INFO(this->get_logger(), "Command '%s' is valid", goal->command.c_str());
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } else {
            RCLCPP_WARN(this->get_logger(), "Command '%s' is not valid", goal->command.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&ArmManageServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ArmInterface::Feedback>();
        auto result = std::make_shared<ArmInterface::Result>();

        feedback->progress = 0.0;
        feedback->status = "Starting execution";
        feedback->partial_result = "";
        goal_handle->publish_feedback(feedback);

        std::string command = goal->command;
        auto start_time = std::chrono::steady_clock::now();

        bool success = execute_command(command, goal->params, goal_handle);

        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<float> execution_time = end_time - start_time;

        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Command canceled";
            result->execution_time = execution_time.count();
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        if (success) {
            result->success = true;
            result->message = "Command executed successfully";
            result->execution_time = execution_time.count();
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        } else {
            result->success = false;
            result->message = "Failed to execute command";
            result->execution_time = execution_time.count();
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Goal aborted");
        }
    }

    bool execute_command(const std::string& command, const std::vector<float>& params,
                         const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        auto feedback = std::make_shared<ArmInterface::Feedback>();

        if (!serial_controller_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "[execute_command] Serial port not open, trying to reopen...");
            if (!serial_controller_.open()) {
                RCLCPP_ERROR(this->get_logger(), "[execute_command] Failed to open serial port");
                feedback->progress = 1.0;
                feedback->status = "Failed to open serial port";
                feedback->partial_result = "Serial port unavailable";
                goal_handle->publish_feedback(feedback);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "[execute_command] Serial port reopened successfully");
        }

        if (command == "forward" || command == "前移") {
            return execute_serial_command("#FORWARD!", "Forward movement", goal_handle);
        } else if (command == "backward" || command == "后移") {
            return execute_serial_command("#BACKWARD!", "Backward movement", goal_handle);
        } else if (command == "turnleft" || command == "左转" || command == "请左转" || command == "左移动") {
            return execute_serial_command("#TURNLEFT!", "Turn left", goal_handle);
        } else if (command == "turnright" || command == "右转" || command == "请右转" || command == "右移动") {
            return execute_serial_command("#TURNRIGHT!", "Turn right", goal_handle);
        } else if (command == "stop" || command == "停") {
            return execute_serial_command("$DST!", "Stop", goal_handle);
        } else if (command == "recover" || command == "恢复" || command == "复原") {
            return execute_serial_command("{G0000#000P1500T1500!#001P1500T1500!#002P1500T1500!#003P1500T1500!#004P1500T1500!#005P1500T1500!}", "Recover", goal_handle);
        } else if (command == "lower" || command == "减低" || command == "请降低") {
            return execute_serial_command("#LOWER!", "Lower", goal_handle);
        } else if (command == "higher" || command == "升高" || command == "提起") {
            return execute_serial_command("#HIGHER!", "Higher", goal_handle);
        } else if (command == "grip" || command == "抓紧" || command == "夹住" || command == "抓住") {
            return execute_serial_command("#GRIP!", "Grip", goal_handle);
        } else if (command == "loose" || command == "松开" || command == "放开") {
            return execute_serial_command("#LOOSE!", "Loose", goal_handle);
        } else if (command == "move_apple") {
            return execute_move_apple(goal_handle);
        } else if (command == "move_cube") {
            return execute_move_object("cube", params, goal_handle);
        } else if (command == "move_cup") {
            return execute_move_object("cup", params, goal_handle);
        } else if (command == "move_bottle") {
            return execute_move_object("bottle", params, goal_handle);
        } else if (command == "open") {
            return execute_open(params, goal_handle);
        } else if (command == "close") {
            return execute_close(params, goal_handle);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown command: %s", command.c_str());
            return false;
        }
    }

    bool execute_serial_command(const std::string& serial_str, const std::string& status,
                                const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing serial command: %s -> %s", status.c_str(), serial_str.c_str());
        
        auto feedback = std::make_shared<ArmInterface::Feedback>();
        
        feedback->progress = 0.1;
        feedback->status = "Checking serial port";
        feedback->partial_result = status;
        goal_handle->publish_feedback(feedback);

        if (!serial_controller_.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Serial port not open, trying to reopen...");
            if (!serial_controller_.open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
                feedback->progress = 1.0;
                feedback->status = "Failed to open serial port";
                feedback->partial_result = "Serial port unavailable";
                goal_handle->publish_feedback(feedback);
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Serial port reopened successfully");
        }
        
        feedback->progress = 0.3;
        feedback->status = "Sending command";
        feedback->partial_result = status;
        goal_handle->publish_feedback(feedback);

        bool success = serial_controller_.send_command(serial_str);
        
        feedback->progress = 1.0;
        feedback->status = success ? "Command sent" : "Failed to send command";
        feedback->partial_result = serial_str;
        goal_handle->publish_feedback(feedback);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Serial command sent successfully: %s", serial_str.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send serial command: %s", serial_str.c_str());
        }
        
        return success;
    }

    bool execute_move_apple(const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        int time_ms = 2000;

        RCLCPP_INFO(this->get_logger(), "Executing move apple command");
        
        auto feedback = std::make_shared<ArmInterface::Feedback>();

        feedback->progress = 0.05;
        feedback->status = "Waiting for apple pose";
        feedback->partial_result = "Subscribing to /apple_pose topic";
        goal_handle->publish_feedback(feedback);

        if (!got_apple_pose_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for apple pose from /apple_pose topic...");
            int wait_count = 0;
            while (!got_apple_pose_ && wait_count < 50) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                rclcpp::spin_some(this->get_node_base_interface());
                wait_count++;
            }
        }

        if (!got_apple_pose_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get apple pose from /apple_pose topic");
            feedback->progress = 1.0;
            feedback->status = "Failed";
            feedback->partial_result = "No apple pose received";
            goal_handle->publish_feedback(feedback);
            return false;
        }

        double camera_x = latest_apple_x_;
        double camera_y = latest_apple_y_;
        double camera_z = latest_apple_z_;
        
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        transform_camera_to_arm(camera_x, camera_y, camera_z, x, y, z);

        RCLCPP_INFO(this->get_logger(), "Apple position in arm coordinates: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

        /////////////////////步骤一：张开夹抓///////////////////////
        bool success = set_servo(serial_controller_, 5, 1000, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
            return false;
        }
        feedback->progress = 0.1;
        feedback->status = "Opening gripper";
        feedback->partial_result = "Target: (gripper open-1000)";
        goal_handle->publish_feedback(feedback);

        
        /////////////////////步骤二：移动到指定位置上方///////////////////////
        success = move_to(serial_controller_, kinematics_, x, y, z, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to apple position");
            return false;
        }

        feedback->progress = 0.4;
        feedback->status = "Moving to apple position";
        feedback->partial_result = "Target: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        goal_handle->publish_feedback(feedback);


        /////////////////////步骤三：关闭夹抓///////////////////////
        feedback->progress = 0.5;
        feedback->status = "Closing gripper";
        feedback->partial_result = "Target: (gripper close-1750)";
        goal_handle->publish_feedback(feedback);
        success = set_servo(serial_controller_, 5, 1750, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to close gripper");
            return false;
        }

        /////////////////////步骤四：提起///////////////////////
        z = z + 160.0;
        success = move_to(serial_controller_, kinematics_, x, y, z, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to apple position");
            return false;
        }

        feedback->progress = 0.6;
        feedback->status = "Moving to lift position";
        feedback->partial_result = "Target: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        goal_handle->publish_feedback(feedback);

        /////////////////////步骤五：移动到另外一边///////////////////////
        x = -x;
        success = move_to(serial_controller_, kinematics_, x, y, z, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to other position");
            return false;
        }

        feedback->progress = 0.7;
        feedback->status = "Moving to other position";
        feedback->partial_result = "Target: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        goal_handle->publish_feedback(feedback);

        /////////////////////步骤六：下落///////////////////////
        z = z -100;
        success = move_to(serial_controller_, kinematics_, x, y, z, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to lift position");
            return false;
        }

        feedback->progress = 0.8;
        feedback->status = "Moving to lift position";
        feedback->partial_result = "Target: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
        goal_handle->publish_feedback(feedback);


        /////////////////////步骤七：张开夹抓///////////////////////
        success = set_servo(serial_controller_, 5, 1000, time_ms);
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
            return false;
        }
        feedback->progress = 1.0;
        feedback->status = "Opening gripper";
        feedback->partial_result = "Target: (gripper open-1000)";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Move object command completed");
        return true;
    }

    bool execute_move_object(const std::string& object, const std::vector<float>& params,
                             const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing move object: %s", object.c_str());
        
        auto feedback = std::make_shared<ArmInterface::Feedback>();
        std::vector<std::string> phases = {"Approaching", "Grasping", "Lifting", "Moving", "Placing", "Releasing"};
        
        for (size_t i = 0; i < phases.size(); ++i) {
            if (goal_handle->is_canceling()) {
                return false;
            }

            feedback->progress = (i + 1) / static_cast<float>(phases.size());
            feedback->status = phases[i] + " " + object;
            feedback->partial_result = "Executing phase " + std::to_string(i + 1) + "/" + std::to_string(phases.size());
            goal_handle->publish_feedback(feedback);

            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }

        RCLCPP_INFO(this->get_logger(), "Move object %s completed", object.c_str());
        return true;
    }

    bool execute_open(const std::vector<float>& params,
                      const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing open command");
        
        auto feedback = std::make_shared<ArmInterface::Feedback>();
        
        feedback->progress = 0.3;
        feedback->status = "Opening";
        feedback->partial_result = "Starting opening sequence";
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        feedback->progress = 0.7;
        feedback->status = "Opening";
        feedback->partial_result = "Opening in progress";
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        feedback->progress = 1.0;
        feedback->status = "Opened";
        feedback->partial_result = "Open completed";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Open command completed");
        return true;
    }

    bool execute_close(const std::vector<float>& params,
                       const std::shared_ptr<GoalHandleArmInterface> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing close command");
        
        auto feedback = std::make_shared<ArmInterface::Feedback>();
        
        feedback->progress = 0.3;
        feedback->status = "Closing";
        feedback->partial_result = "Starting closing sequence";
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        feedback->progress = 0.7;
        feedback->status = "Closing";
        feedback->partial_result = "Closing in progress";
        goal_handle->publish_feedback(feedback);

        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        feedback->progress = 1.0;
        feedback->status = "Closed";
        feedback->partial_result = "Close completed";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Close command completed");
        return true;
    }

    bool is_valid_command(const std::string& command)
    {
        const std::vector<std::string> valid_commands = {
            "forward", "backward", "turnleft", "turnright", "stop",
            "move_apple", "move_cube", "move_cup", "move_bottle",
            "open", "close", "recover", "lower", "higher", "grip", "loose",
            "前移", "后移", "左转", "请左转", "左移动", "右转", "请右转", "右移动",
            "停", "恢复", "复原", "减低", "请降低", "升高", "提起",
            "抓紧", "夹住", "抓住", "松开", "放开"
        };
        
        return std::find(valid_commands.begin(), valid_commands.end(), command) != valid_commands.end();
    }
};

int main(int argc, char ** argv)
{
    RCLCPP_INFO(rclcpp::get_logger("arm_manage_server"), "Starting arm_manage_server...");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmManageServer>();
    RCLCPP_INFO(rclcpp::get_logger("arm_manage_server"), "arm_manage_server started successfully, waiting for commands...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}