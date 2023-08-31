#ifndef hiros_xsens_mtw_wrapper_h
#define hiros_xsens_mtw_wrapper_h

// Standard dependencies
#include <csignal>
#include <deque>

// ROS dependencies
// #include "geometry_msgs/msg/Vector3Stamped.h"
// #include "ros/ros.h"
// #include "sensor_msgs/msg/FluidPressure.h"
// #include "sensor_msgs/msg/Imu.h"
// #include "sensor_msgs/msg/MagneticField.h"
// #include "std_msgs/Header.h"
// #include "tf2_ros/transform_broadcaster.h"

// Internal dependencies ROS1
// #include "hiros_xsens_mtw_wrapper/Euler.h"
// #include "hiros_xsens_mtw_wrapper/MIMU.h"
// #include "hiros_xsens_mtw_wrapper/MIMUArray.h"
// #include "hiros_xsens_mtw_wrapper/ResetOrientation.h"
// #include "xsens_mtw/MtwCallback.h"
// #include "xsens_mtw/Synchronizer.h"
// #include "xsens_mtw/WirelessMasterCallback.h"



// ROS2 dependencies
// #include "geometry_msgs/msg/Vector3Stamped.h"
// #include "ros/ros.h"
// #include "sensor_msgs/msg/FluidPressure.h"
// #include "sensor_msgs/msg/Imu.h"
// #include "sensor_msgs/msg/MagneticField.h"
// #include "std_msgs/Header.h"
// #include "tf2_ros/transform_broadcaster.h"


// Internal dependencies ROS2

#include "vi_interfaces/msg/euler.hpp"
#include "vi_interfaces/msg/mimu.hpp"
#include "vi_interfaces/msg/mimu_array.hpp"

#include "vi_interfaces/srv/reset_orientation.hpp"


#include "xsens_mtw/MtwCallback.h"
#include "xsens_mtw/Synchronizer.h"
#include "xsens_mtw/WirelessMasterCallback.h"


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "xsensdeviceapi.h"



#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace xsens_mtw {

    static const std::map<std::string, Synchronizer::SyncPolicy> sync_policy_map = {
      {"fill_partial_frames", Synchronizer::SyncPolicy::fillPartialFrames},
      {"skip_partial_frames", Synchronizer::SyncPolicy::skipPartialFrames}};

    struct XsensMtwParameters
    {
      int desired_update_rate;
      int desired_radio_channel;

      bool reset_initial_orientation;
    };

    struct WrapperParameters
    {
      std::string tf_prefix;
      bool enable_custom_labeling;

      bool synchronize;
      std::string sync_policy_name;
      bool publish_mimu_array;

      bool publish_imu;
      bool publish_mag;
      bool publish_euler;
      bool publish_free_acceleration;
      bool publish_pressure;
      bool publish_tf;
    };

    class Wrapper : public rclcpp::Node
    {
    public:
      Wrapper();
      ~Wrapper() {}

      void start();
      void run();

    private:
      bool configure();
      void stop();

      void configureWrapper();
      bool configureXsensMtw();

      void stopXsensMtw();
      void stopWrapper();

      bool waitMtwConnection();
      bool getMtwsDeviceIstances();
      void attachCallbackHandlers();
      bool startMeasurement();

      bool constructControl();
      bool findWirelessMaster();
      bool openPort();
      bool getXsdeviceInstance();
      bool setConfigMode();
      void attachCallbackHandler();
      bool getClosestUpdateRate();
      bool setUpdateRate();
      bool setRadioChannel();
      bool disableRadio();

      void setupRos();
      bool resetInitialOrientation() const;
      void syncInitialPackets();

      std::string getDeviceLabel(const XsDeviceId& t_id) const;
      XsDeviceId getDeviceId(const std::string t_label) const;
      std::string composeTopicPrefix(const XsDeviceId& t_id) const;

      void publishPacket(const std::shared_ptr<XsDataPacket>& t_packet);
      void publishFrame(const std::vector<std::shared_ptr<XsDataPacket>>& t_frame);

      std_msgs::msg::Header getHeader(const std::shared_ptr<XsDataPacket>& t_packet) const;
      sensor_msgs::msg::Imu getImuMsg(const std::shared_ptr<XsDataPacket>& t_packet) const;
      sensor_msgs::msg::MagneticField getMagMsg(const std::shared_ptr<XsDataPacket>& t_packet) const;
      vi_interfaces::msg::Euler getEulerMsg(const std::shared_ptr<XsDataPacket>& t_packet) const;
      geometry_msgs::msg::Vector3Stamped getFreeAccelerationMsg(const std::shared_ptr<XsDataPacket>& t_packet) const;
      sensor_msgs::msg::FluidPressure getPressureMsg(const std::shared_ptr<XsDataPacket>& t_packet) const;
      vi_interfaces::msg::MIMU getMIMUMsg(const std::shared_ptr<XsDataPacket>& t_packet) const;
      vi_interfaces::msg::MIMUArray getMIMUArrayMsg(const std::vector<std::shared_ptr<XsDataPacket>>& t_frame) const;
      geometry_msgs::msg::TransformStamped getTf(const std::shared_ptr<XsDataPacket>& t_packet) const;

      // geometry_msgs::msg::TransformStamped

      bool resetOrientation(vi_interfaces::srv::ResetOrientation::Request& t_req,
                            vi_interfaces::srv::ResetOrientation::Response& t_res);

      static inline void sighandler(int t_sig) { s_request_shutdown = (t_sig == SIGINT); };

      XsensMtwParameters m_mtw_params;
      WrapperParameters m_wrapper_params;

      WirelessMasterCallback m_wireless_master_callback;
      std::map<XsDeviceId, MtwCallback*> m_mtw_callbacks;

      XsControl* m_control;
      XsPortInfoArray m_detected_devices;
      XsPortInfoArray::const_iterator m_wireless_master_port;
      XsDevicePtr m_wireless_master_device;

      XsIntArray m_supported_update_rates;
      int m_update_rate;

      const unsigned int m_connection_timeout = 5000; // [ms]
      unsigned long m_number_of_connected_mtws;

      std::map<XsDeviceId, XsDevicePtr> m_connected_devices;
      std::map<XsDeviceId, std::string> m_ids_to_labels;
      std::map<std::string, XsDeviceId> m_labels_to_ids;

      long m_initial_packet_id;
      // ros::Time m_initial_timestamp;
      rclcpp::Time m_initial_timestamp;


      bool m_xsens_mtw_configured;

      // ros::NodeHandle m_nh;

      std::string m_node_namespace;

      const unsigned int m_ros_topic_queue_size = 10;

      // ros::ServiceServer m_reset_orientation_srv;
      rclcpp::Service<vi_interfaces::srv::ResetOrientation>::SharedPtr m_reset_orientation_srv;


      //ros::Publisher m_data_pub;
      rclcpp::Publisher<vi_interfaces::msg::MIMUArray>::SharedPtr m_data_pub;

      // std::map<XsDeviceId, ros::Publisher> m_imu_pubs;
      // std::map<XsDeviceId, ros::Publisher> m_mag_pubs;
      // std::map<XsDeviceId, ros::Publisher> m_euler_pubs;
      // std::map<XsDeviceId, ros::Publisher> m_free_acceleration_pubs;
      // std::map<XsDeviceId, ros::Publisher> m_pressure_pubs;

      std::map<XsDeviceId, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> m_imu_pubs;
      std::map<XsDeviceId, rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr> m_mag_pubs;
      std::map<XsDeviceId, rclcpp::Publisher<vi_interfaces::msg::Euler>::SharedPtr> m_euler_pubs;
      std::map<XsDeviceId, rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr> m_free_acceleration_pubs;
      std::map<XsDeviceId, rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr> m_pressure_pubs;
      

      // tf2_ros::TransformBroadcaster m_tf_broadcaster;

      std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

      static bool s_request_shutdown;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
