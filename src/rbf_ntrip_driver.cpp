#include "rbf_ntrip_driver/rbf_ntrip_driver.h"

namespace rbf_ntrip_driver {

    NtripDriver::NtripDriver(const rclcpp::NodeOptions& options) : Node("rbf_ntrip_driver", options)
    {
        load_parameters();
        ntrip_client_.OnReceived(std::bind(&NtripDriver::ntrip_client_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        if (config_.serial_port.publish_port_rtcm) {
            try{
                serial_port_.set_port_name(config_.serial_port.port.c_str());
                serial_port_.open();
                serial_port_.configure(config_.serial_port.baudrate, 8, 'N', 1);
            }
            catch (const SerialPortException& e){
                RCLCPP_ERROR(get_logger(), e.what());
                rclcpp::shutdown();
            }
        }

        if (config_.rtcm_publisher.publish_rtcm) {
            pub_rtcm_ = this->create_publisher<mavros_msgs::msg::RTCM>(config_.rtcm_publisher.topic_name, 10);
        }

        if(config_.ntrip.use_nav_sat_fix_init){
            sub_nav_sat_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                config_.ntrip.nav_sat_fix_topic_name, 10, std::bind(&NtripDriver::nav_sat_fix_callback, this, std::placeholders::_1));
        }
        if(!config_.ntrip.use_nav_sat_fix_init){
            try_to_ntrip_connect();
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NtripDriver::timer_callback, this));
        }
    }

    void NtripDriver::load_parameters()
    {
        config_.ntrip.host = declare_parameter("ntrip.host", "ntrip.example.com");
        config_.ntrip.port = declare_parameter("ntrip.port", 2101);
        config_.ntrip.mountpoint = declare_parameter("ntrip.mount_point", "VRSRTCM31");
        config_.ntrip.username = declare_parameter("ntrip.user_name", "username");
        config_.ntrip.password = declare_parameter("ntrip.password", "password");
        config_.ntrip.use_nav_sat_fix_init = declare_parameter("ntrip.use_nav_sat_fix_init", false);
        config_.ntrip.nav_sat_fix_topic_name = declare_parameter("ntrip.nav_sat_fix_topic_name", "/fix");
        config_.ntrip.init_lat_position = declare_parameter("ntrip.init_ntrip_location_lat", 0.0);
        config_.ntrip.init_lon_position = declare_parameter("ntrip.init_ntrip_location_lon", 0.0);

        config_.serial_port.port = declare_parameter("serial_port.name", "/dev/ttyUSB0");
        config_.serial_port.baudrate = declare_parameter("serial_port.baud_rate", 9600);
        config_.serial_port.publish_port_rtcm = declare_parameter("serial_port.publish_port_rtcm", false);

        config_.rtcm_publisher.topic_name = declare_parameter("rtcm_publisher.rtcm_topic", "rtcm");
        config_.rtcm_publisher.publish_rtcm = declare_parameter("rtcm_publisher.publish_rtcm", true);
        config_.rtcm_publisher.frame_id = declare_parameter("rtcm_publisher.frame_id", "rtcm");

        RCLCPP_INFO(this->get_logger(),"---------NTRIP CONFIGURATION--------");
        RCLCPP_INFO(this->get_logger(),"host: %s", config_.ntrip.host.c_str());
        RCLCPP_INFO(this->get_logger(),"port: %d", config_.ntrip.port);
        RCLCPP_INFO(this->get_logger(),"mountpoint: %s", config_.ntrip.mountpoint.c_str());
        RCLCPP_INFO(this->get_logger(),"username: %s", config_.ntrip.username.c_str());
        RCLCPP_INFO(this->get_logger(),"password: %s", config_.ntrip.password.c_str());
        RCLCPP_INFO(this->get_logger(),"use_nav_sat_fix_init: %d", config_.ntrip.use_nav_sat_fix_init);
        RCLCPP_INFO(this->get_logger(),"nav_sat_fix_topic_name: %s", config_.ntrip.nav_sat_fix_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(),"init_lat_position: %f", config_.ntrip.init_lat_position);
        RCLCPP_INFO(this->get_logger(),"init_lon_position: %f", config_.ntrip.init_lon_position);
        RCLCPP_INFO(this->get_logger(),"---------SERIAL PORT CONFIGURATION--------");
        RCLCPP_INFO(this->get_logger(),"port: %s", config_.serial_port.port.c_str());
        RCLCPP_INFO(this->get_logger(),"baudrate: %d", config_.serial_port.baudrate);
        RCLCPP_INFO(this->get_logger(),"publish_port_rtcm: %d", config_.serial_port.publish_port_rtcm);
        RCLCPP_INFO(this->get_logger(),"---------RTCM PUBLISHER CONFIGURATION--------");
        RCLCPP_INFO(this->get_logger(),"topic_name: %s", config_.rtcm_publisher.topic_name.c_str());
        RCLCPP_INFO(this->get_logger(),"publish_rtcm: %d", config_.rtcm_publisher.publish_rtcm);
        RCLCPP_INFO(this->get_logger(),"frame_id: %s", config_.rtcm_publisher.frame_id.c_str());

        RCLCPP_INFO(this->get_logger(),"----------------------------------");
    }

    bool NtripDriver::init_ntrip(){
        ntrip_client_.Init(config_.ntrip.host, config_.ntrip.port, config_.ntrip.username, config_.ntrip.password, config_.ntrip.mountpoint);
        if(!config_.ntrip.use_nav_sat_fix_init){
            ntrip_client_.set_location(config_.ntrip.init_lat_position, config_.ntrip.init_lon_position);
        }
        return ntrip_client_.Run();
    }

    void NtripDriver::try_to_ntrip_connect(){
        int try_count = 0;
        while (try_count < 10 && rclcpp::ok()) {
            if (init_ntrip()) {
                RCLCPP_INFO(get_logger(), "NTRIP client initialized successfully");
                break;
            }
            try_count++;
            RCLCPP_INFO(get_logger(), "Failed to initialize[%d] NTRIP client  , retrying in 1 second", try_count);
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        if (try_count >= 10) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize NTRIP client after 10 attempts");
            rclcpp::shutdown();
        }
    }

    void NtripDriver::timer_callback()
    {
        if(!ntrip_client_.service_is_running()){
            ntrip_client_.Stop();
            RCLCPP_ERROR(this->get_logger(), "NTRIP client is not running");
            try_to_ntrip_connect();
        }
    }

    void NtripDriver::ntrip_client_callback(char const* _buffer, int _size){
        auto rtcm_msg = mavros_msgs::msg::RTCM();
        rtcm_msg.header.stamp = this->now();
        rtcm_msg.header.frame_id = config_.rtcm_publisher.frame_id;
        rtcm_msg.data = std::vector<uint8_t>(_buffer, _buffer + _size);
        if(config_.rtcm_publisher.publish_rtcm){
            pub_rtcm_->publish(rtcm_msg);
        }
        if(config_.serial_port.publish_port_rtcm){
            serial_port_.write(_buffer, _size);
        }
    }

    void NtripDriver::nav_sat_fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
        ntrip_client_.set_location(msg->latitude, msg->longitude);
        RCLCPP_INFO(get_logger(), "NavSatFix message received, NTRIP client initialized successfully");
        try_to_ntrip_connect();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&NtripDriver::timer_callback, this));
        sub_nav_sat_fix_.reset();
    }

}; // namespace rbf_ntrip_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbf_ntrip_driver::NtripDriver)
