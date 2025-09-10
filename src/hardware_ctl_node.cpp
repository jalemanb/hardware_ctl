#include <rclcpp/rclcpp.hpp>
#include <person_detection_msgs/msg/candidate.hpp>
#include <person_detection_msgs/msg/candidate_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

using namespace std::chrono_literals;

class MultifacetedCameraTiltController : public rclcpp::Node
{
public:
    MultifacetedCameraTiltController()
        : Node("multifaceted_camera_tilt_controller"),
          tilt_signal_(50),
          current_measurement(0.0),
          last_float_msg_time_(this->now())
    {
        candidates_sub_ = this->create_subscription<person_detection_msgs::msg::CandidateArray>(
            "image_detections", 10,
            std::bind(&MultifacetedCameraTiltController::poseCallback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&MultifacetedCameraTiltController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Multifaceted Camera Tilt Controller Node Initialized");

        serialSetup(device_, baudrate_);
    }

    ~MultifacetedCameraTiltController()
    {
        if (serial_port_.IsOpen())
        {
            serial_port_.Close();
        }
    }

private:
    rclcpp::Subscription<person_detection_msgs::msg::CandidateArray>::SharedPtr candidates_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    int tilt_signal_;
    double current_measurement;
    double servo_angle_;
    rclcpp::Time last_float_msg_time_;
    int Init;
    int status;

    std::string device_ = "/dev/ttyACM0";
    int baudrate_ = 9600;
    LibSerial::SerialPort serial_port_;

    void serialSetup(const std::string &port_name, int baud_rate)
    {
        try
        {
            serial_port_.Open(port_name);
            serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
            serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
            serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

            RCLCPP_INFO(this->get_logger(), "Serial port %s initialized with baud rate %d.", port_name.c_str(), baud_rate);
        }
        catch (const LibSerial::OpenFailed &)
        {
            throw std::runtime_error("Failed to open serial port " + port_name);
        }
    }

    void writeIntASCII(int value)
    {
        if (!serial_port_.IsOpen())
        {
            RCLCPP_ERROR(this->get_logger(), "Serial port not open.");
            return;
        }

        std::string msg = std::to_string(value) + "\n";
        try
        {
            serial_port_.Write(msg);
        }
        catch (const LibSerial::NotOpen &)
        {
            RCLCPP_ERROR(this->get_logger(), "Attempted to write to a closed serial port.");
        }
    }

    void poseCallback(const person_detection_msgs::msg::CandidateArray::SharedPtr msg)
    {
        double min_distance = 1000000.0;

        for (const auto &candidate : msg->candidates)
        {
            double person_dist = candidate.dist;
            if (person_dist < min_distance)
            {
                min_distance = person_dist;
            }

            current_measurement = (candidate.conf < 0.5) ? 0.0 : candidate.v;
        }
    }

    void controlLoop()
    {
        int current_measurement_int = static_cast<int>(current_measurement);
        // std::cout << "current_measurement: " << current_measurement << std::endl;
        writeIntASCII(current_measurement_int);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultifacetedCameraTiltController>());
    rclcpp::shutdown();
    return 0;
}
