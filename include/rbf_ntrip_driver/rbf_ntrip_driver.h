#ifndef RBF_NTRIP_DRIVER_H_
#define RBF_NTRIP_DRIVER_H_

#include <rclcpp/rclcpp.hpp>
#include <rbf_ntrip_driver/serial_port.h>
#include <ntrip/ntrip_client.h>

#include <mavros_msgs/msg/rtcm.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace rbf_ntrip_driver {

class NtripDriver : public rclcpp::Node {
public:
    NtripDriver(const rclcpp::NodeOptions & options);
    ~NtripDriver() override{
        serial_port_.close();
        ntrip_client_.Stop();
    }

    struct Config{
        struct Ntrip{
            std::string host;
            int port;
            std::string mountpoint;
            std::string username;
            std::string password;
            bool use_nav_sat_fix_init;
            std::string nav_sat_fix_topic_name;
            double init_lat_position;
            double init_lon_position;
        };
        struct SerialPort{
            std::string port;
            int baudrate;
            bool publish_port_rtcm;
        };
        struct RTCMPublisher{
            std::string topic_name;
            std::string frame_id;
            bool publish_rtcm;
        };
        Ntrip ntrip;
        SerialPort serial_port;
        RTCMPublisher rtcm_publisher;
    };
    Config config_;
private:
    SerialPort serial_port_;
    libntrip::NtripClient ntrip_client_;
    rclcpp::TimerBase::SharedPtr timer_;


    /*LOAD PARAMETERS*/
    void load_parameters();
    /*INIT NTRIP*/
    bool init_ntrip();
    void try_to_ntrip_connect();

    /*PUBLISHERS*/
    rclcpp::Publisher<mavros_msgs::msg::RTCM>::SharedPtr pub_rtcm_;

    /*SUBSCRIBERS*/
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_nav_sat_fix_;

    /*CALLBACKS*/
    void timer_callback();
    void ntrip_client_callback(char const* _buffer, int _size);
    void nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

} // namespace rbf_ntrip_driver

#endif // RBF_NTRIP_DRIVER_HPP_

