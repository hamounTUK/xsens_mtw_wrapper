// Standard dependencies
#include <numeric>

// Internal dependencies
#include "xsens_mtw/Wrapper.h"

#include "xstypes/xstime.h"


bool hiros::xsens_mtw::Wrapper::s_request_shutdown = false;

// hiros::xsens_mtw::Wrapper::Wrapper()
//   : m_number_of_connected_mtws(0)
//   , m_xsens_mtw_configured(false)
//   , m_nh("~")
//   , m_node_namespace(m_nh.getNamespace())
// {

//   struct sigaction sig_act;
//   memset(&sig_act, 0, sizeof(sig_act));
//   sig_act.sa_handler = hiros::xsens_mtw::Wrapper::sighandler;
//   sigaction(SIGINT, &sig_act, nullptr);
// }


hiros::xsens_mtw::Wrapper::Wrapper()
  : Node("hiros_xsens")
{

  this->declare_parameter<bool>("xsens_mtw_node_required", false);
  this->declare_parameter<std::string>("node_name", "xsens_mtw");
  this->declare_parameter<std::string>("tf_prefix", "");

  this->declare_parameter<int16_t>("desired_update_rate", 80);
  this->declare_parameter<int16_t>("desired_radio_channel", 21);

  this->declare_parameter<bool>("reset_initial_orientation", false);
  this->declare_parameter<bool>("enable_custom_labeling", false);

  this->declare_parameter<bool>("synchronize", false);
  this->declare_parameter<std::string>("sync_policy", "skip_partial_frames");
  this->declare_parameter<bool>("publish_mimu_array", false);

  this->declare_parameter<bool>("publish_imu", false);
  this->declare_parameter<bool>("publish_mag", false);
  this->declare_parameter<bool>("publish_euler", false);
  this->declare_parameter<bool>("publish_free_acceleration", false);
  this->declare_parameter<bool>("publish_pressure", false);
  this->declare_parameter<bool>("publish_tf", false);


  m_number_of_connected_mtws = 0;
  m_xsens_mtw_configured = false;
  m_node_namespace = this->get_namespace();
  m_node_namespace = "mtw";


  RCLCPP_INFO_STREAM(this->get_logger(), "NODE NAME::::::::::::::: " << m_node_namespace);


  struct sigaction sig_act;
  memset(&sig_act, 0, sizeof(sig_act));


  sig_act.sa_handler = hiros::xsens_mtw::Wrapper::sighandler;


  sigaction(SIGINT, &sig_act, nullptr);


  start();

std::cout<< "DON WITH START";

  run();


}




void hiros::xsens_mtw::Wrapper::start()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Starting");

  do {
    if (!m_xsens_mtw_configured) {

      if (!configure()) {
        // ros::shutdown();

        rclcpp::shutdown();
      }


    }
    else {
      RCLCPP_WARN_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Trying to reduce the update rate");


      stopXsensMtw();

      unsigned long up_rate_index = static_cast<unsigned long>(m_supported_update_rates.find(m_update_rate));

      if (up_rate_index == m_supported_update_rates.size() - 1) {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to go to measurement mode");

        // ros::shutdown();
        rclcpp::shutdown();
      }

      m_mtw_params.desired_update_rate = m_supported_update_rates.at(++up_rate_index);

      configureXsensMtw();
    }

    if (!waitMtwConnection()) {
      // ros::shutdown();
      rclcpp::shutdown();
    }

    if (!getMtwsDeviceIstances()) {
      // ros::shutdown();
      rclcpp::shutdown();

    }

    attachCallbackHandlers();
  } while (!startMeasurement());

  setupRos();

  if (m_mtw_params.reset_initial_orientation) {
    resetInitialOrientation();
  }

  syncInitialPackets();
}

void hiros::xsens_mtw::Wrapper::run()
{
  RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "Xsens Mtw Wrapper... RUNNING" << BASH_MSG_RESET);


  std::unique_ptr<Synchronizer> sync;
  if (m_wrapper_params.synchronize) {
    sync = std::make_unique<Synchronizer>(m_mtw_callbacks, sync_policy_map.at(m_wrapper_params.sync_policy_name));
  }


  while (rclcpp::ok() && !s_request_shutdown) {
    for (auto& device : m_connected_devices) {
      if (m_mtw_callbacks.at(device.first)->newDataAvailable()) {

        if (m_wrapper_params.synchronize) {

          sync->add(m_mtw_callbacks.at(device.first)->getLatestPacket());
        }
        else {


          publishPacket(m_mtw_callbacks.at(device.first)->getLatestPacket());

          RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "PUB" << BASH_MSG_RESET);
          m_mtw_callbacks.at(device.first)->deleteOldestPacket();

        }
      }

      if (m_wrapper_params.synchronize) {
        if (sync->newFrameAvailable()) {
          publishFrame(sync->getLatestFrame());
          sync->clearLatestFrame();
        }
      }
    }
  }

  stop();
}

bool hiros::xsens_mtw::Wrapper::configure()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Configuring");




  if (m_xsens_mtw_configured) {
    m_xsens_mtw_configured = false;
    stop();
  }

  configureWrapper();

  m_xsens_mtw_configured = configureXsensMtw();



  if (m_xsens_mtw_configured) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... CONFIGURED");

  }

  return m_xsens_mtw_configured;
}

void hiros::xsens_mtw::Wrapper::stop()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Stopping");

  stopXsensMtw();
  stopWrapper();

  RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "Xsens Mtw Wrapper... STOPPED" << BASH_MSG_RESET);

  // ros::shutdown();
  rclcpp::shutdown();

}

void hiros::xsens_mtw::Wrapper::configureWrapper()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Configuring Wrapper");


  // m_nh.getParam("desired_update_rate", m_mtw_params.desired_update_rate);
  // m_nh.getParam("desired_radio_channel", m_mtw_params.desired_radio_channel);

  // m_nh.getParam("reset_initial_orientation", m_mtw_params.reset_initial_orientation);

  // m_nh.getParam("tf_prefix", m_wrapper_params.tf_prefix);
  // m_nh.getParam("enable_custom_labeling", m_wrapper_params.enable_custom_labeling);

  // m_nh.getParam("synchronize", m_wrapper_params.synchronize);
  // m_nh.getParam("sync_policy", m_wrapper_params.sync_policy_name);


  this->get_parameter("desired_update_rate", m_mtw_params.desired_update_rate);
  this->get_parameter("desired_radio_channel", m_mtw_params.desired_radio_channel);
  this->get_parameter("reset_initial_orientation", m_mtw_params.reset_initial_orientation);
  this->get_parameter("tf_prefix", m_wrapper_params.tf_prefix);
  this->get_parameter("enable_custom_labeling", m_wrapper_params.enable_custom_labeling);
  this->get_parameter("synchronize", m_wrapper_params.synchronize);
  this->get_parameter("sync_policy", m_wrapper_params.sync_policy_name);


RCLCPP_INFO_STREAM(this->get_logger(), "### update rate: " << m_mtw_params.desired_update_rate);


  if (sync_policy_map.find(m_wrapper_params.sync_policy_name) == sync_policy_map.end()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Sync policy " << m_wrapper_params.sync_policy_name << " is not supported");

    std::string supported_policies;
    for (auto& policy : sync_policy_map) {
      supported_policies += (policy.first + ", ");
    }
    
    RCLCPP_WARN_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Supported policies: " << supported_policies.erase(supported_policies.length() - 2));

    RCLCPP_WARN_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Using " << sync_policy_map.rbegin()->first << " policy");

    m_wrapper_params.sync_policy_name = sync_policy_map.rbegin()->first;
  }
  this->get_parameter("publish_mimu_array", m_wrapper_params.publish_mimu_array);

  if (m_wrapper_params.publish_mimu_array && !m_wrapper_params.synchronize) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Cannot publish MIMU array messages when Synchronizer is off. Closing");

    // ros::shutdown();
    rclcpp::shutdown();

  }
// ROS1
  // m_nh.getParam("publish_imu", m_wrapper_params.publish_imu);
  // m_nh.getParam("publish_mag", m_wrapper_params.publish_mag);
  // m_nh.getParam("publish_euler", m_wrapper_params.publish_euler);
  // m_nh.getParam("publish_free_acceleration", m_wrapper_params.publish_free_acceleration);
  // m_nh.getParam("publish_pressure", m_wrapper_params.publish_pressure);
  // m_nh.getParam("publish_tf", m_wrapper_params.publish_tf);


  this->get_parameter("publish_imu", m_wrapper_params.publish_imu);
  this->get_parameter("publish_mag", m_wrapper_params.publish_mag);
  this->get_parameter("publish_euler", m_wrapper_params.publish_euler);
  this->get_parameter("publish_free_acceleration", m_wrapper_params.publish_free_acceleration);
  this->get_parameter("publish_pressure", m_wrapper_params.publish_pressure);
  this->get_parameter("publish_tf", m_wrapper_params.publish_tf);



  bool nothing_to_publish =
    !(m_wrapper_params.publish_imu || m_wrapper_params.publish_mag || m_wrapper_params.publish_euler
      || m_wrapper_params.publish_free_acceleration || m_wrapper_params.publish_pressure);

  if (nothing_to_publish && !m_wrapper_params.publish_tf) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Nothing to publish. Closing");

    // ros::shutdown();
    rclcpp::shutdown();

  }

  // if (m_wrapper_params.enable_custom_labeling) {
  //   XmlRpc::XmlRpcValue xml_sensor_labels;
    
  //   if (this->get_parameter("sensor_labels", xml_sensor_labels)) {

  //     for (int i = 0; i < xml_sensor_labels.size(); ++i) {
  //       m_ids_to_labels.emplace(utils::toXsDeviceId(xml_sensor_labels[i]["imu_id"]), xml_sensor_labels[i]["label"]);
  //       m_labels_to_ids.emplace(xml_sensor_labels[i]["label"], utils::toXsDeviceId(xml_sensor_labels[i]["imu_id"]));
  //     }
  //   }
  // }

  //ROS2
  // if (m_wrapper_params.enable_custom_labeling) {
  //   std::string sensor_config_yaml;
  //   if (this->get_parameter("sensor_labels", sensor_config_yaml)) {


  // }


}

bool hiros::xsens_mtw::Wrapper::configureXsensMtw()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Configuring Xsens Mtw");


  bool success = constructControl();

  success = success && findWirelessMaster();
  success = success && openPort();
  success = success && getXsdeviceInstance();
  success = success && setConfigMode();
  attachCallbackHandler();

  success = success && getClosestUpdateRate();
  success = success && setUpdateRate();
  success = success && setRadioChannel();

  if (!success) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to configure Xsens Mtw");

  }

  return success;
}

void hiros::xsens_mtw::Wrapper::stopXsensMtw()
{
  if (!setConfigMode()) {
    // ros::shutdown();
    rclcpp::shutdown();

  }

  if (!disableRadio()) {
    // ros::shutdown();
    rclcpp::shutdown();

  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Closing XsControl");

  m_control->close();
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Deleting MTW callbacks");

  for (auto& mtw_callback : m_mtw_callbacks) {
    delete (mtw_callback.second);
  }
  m_mtw_callbacks.clear();

  
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Clearing MTW devices");

  m_connected_devices.clear();
}

void hiros::xsens_mtw::Wrapper::stopWrapper()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Shutting down ROS publisher(s)");

  if (m_wrapper_params.publish_mimu_array) {
    if (m_data_pub) {
      // m_data_pub.shutdown();
      // No direct equivalent in Ros2

      //Ros2
      delete &m_data_pub;
    }
  }
  else {
    if (m_wrapper_params.publish_imu) {
      for (auto& pub : m_imu_pubs) {
        if (pub.second) {
          // pub.second.shutdown();
          // No direct equivalent in Ros2

          //Ros2
          delete &pub;
        }
      }
    }

    if (m_wrapper_params.publish_mag) {
      for (auto& pub : m_mag_pubs) {
        if (pub.second) {
          // pub.second.shutdown();
          // No direct equivalent in Ros2
          
          //Ros2
          delete &pub;

        }
      }
    }

    if (m_wrapper_params.publish_euler) {
      for (auto& pub : m_euler_pubs) {
        if (pub.second) {
          // pub.second.shutdown();
          // No direct equivalent in Ros2

          //Ros2
          delete &pub;
        }
      }
    }

    if (m_wrapper_params.publish_free_acceleration) {
      for (auto& pub : m_free_acceleration_pubs) {
        if (pub.second) {
          // pub.second.shutdown();
          // No direct equivalent in Ros2

          //Ros2
          delete &pub;
        }
      }
    }

    if (m_wrapper_params.publish_pressure) {
      for (auto& pub : m_pressure_pubs) {
        if (pub.second) {
          // pub.second.shutdown();
          // No direct equivalent in Ros2

          //Ros2
          delete &pub;
        }
      }
    }
  }


  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Shutting down ROS service server");
  // m_reset_orientation_srv.shutdown();
  delete &m_reset_orientation_srv;


  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Clearing maps");

  m_ids_to_labels.clear();
  m_labels_to_ids.clear();
}

bool hiros::xsens_mtw::Wrapper::waitMtwConnection()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Waiting for MTW to wirelessly connect");


  XsTime::msleep(m_connection_timeout);

  m_number_of_connected_mtws = m_wireless_master_callback.getWirelessMTWs().size();
  
  RCLCPP_INFO_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Number of connected MTWs: " << m_number_of_connected_mtws);


  if (m_number_of_connected_mtws == 0) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to connect to MTWs");

    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getMtwsDeviceIstances()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Getting XsDevice instances for all MTWs");

  for (auto& xs_device_id : m_control->deviceIds()) {
    if (xs_device_id.isMtw()) {
      XsDevicePtr xs_device = m_control->device(xs_device_id);
      if (xs_device != nullptr) {
        m_connected_devices.emplace(xs_device_id, xs_device);
      }
      else {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to create an MTW XsDevice instance");

        return false;
      }
    }
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandlers()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(),"Xsens Mtw Wrapper... Attaching callback handlers to MTWs");

  int mtw_index = 0;
  for (auto& device : m_connected_devices) {
    m_mtw_callbacks.emplace(device.first, new MtwCallback(mtw_index++, device.second));
    device.second->addCallbackHandler(m_mtw_callbacks.at(device.first));
  }
}

bool hiros::xsens_mtw::Wrapper::startMeasurement()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Starting measurement");

  if (!m_wireless_master_device->gotoMeasurement()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to go to measurement mode");

    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::constructControl()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Constructing XsControl");

  m_control = XsControl::construct();

  if (m_control == nullptr) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to construct XsControl instance");

    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::findWirelessMaster()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Scanning ports");


  m_detected_devices = XsScanner::scanPorts();



  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Finding wireless master");

  m_wireless_master_port = m_detected_devices.begin();
  while (m_wireless_master_port != m_detected_devices.end() && !m_wireless_master_port->deviceId().isWirelessMaster()) {
    ++m_wireless_master_port;
  }

  if (m_wireless_master_port == m_detected_devices.end()) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... No wireless masters found");

    return false;
  }

    RCLCPP_INFO_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Wireless master found @ " << *m_wireless_master_port);

  return true;
}

bool hiros::xsens_mtw::Wrapper::openPort()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Opening port");

  if (!m_control->openPort(m_wireless_master_port->portName().toStdString(), m_wireless_master_port->baudrate())) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to open port " << *m_wireless_master_port);

    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getXsdeviceInstance()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Getting XsDevice instance for wireless master");

  m_wireless_master_device = m_control->device(m_wireless_master_port->deviceId());
  if (m_wireless_master_device == nullptr) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to construct XsDevice instance: " << *m_wireless_master_port);

    return false;
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... XsDevice instance created @ " << utils::toString(*m_wireless_master_device));

  return true;
}

bool hiros::xsens_mtw::Wrapper::setConfigMode()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Setting config mode");

  if (!m_wireless_master_device->gotoConfig()) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to go to config mode: " + utils::toString(*m_wireless_master_device));

    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandler()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Attaching callback handler");

  m_wireless_master_device->addCallbackHandler(&m_wireless_master_callback);
}

bool hiros::xsens_mtw::Wrapper::getClosestUpdateRate()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Getting the list of the supported update rates");

  m_supported_update_rates = m_wireless_master_device->supportedUpdateRates();

  std::string info_str = "Xsens Mtw Wrapper... Supported update rates:";
  for (auto& up_rate : m_supported_update_rates) {
    info_str += " " + std::to_string(up_rate);
  }
  RCLCPP_INFO_STREAM(this->get_logger(), info_str);


  if (m_supported_update_rates.empty()) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to get supported update rates");

    return false;
  }

  if (m_supported_update_rates.size() == 1) {
    m_update_rate = m_supported_update_rates.at(0);
  }
  else {
    int u_rate_dist = -1;
    int closest_update_rate = -1;

    for (auto& up_rate : m_supported_update_rates) {
      const int curr_dist = std::abs(up_rate - m_mtw_params.desired_update_rate);

      if ((u_rate_dist == -1) || (curr_dist < u_rate_dist)) {
        u_rate_dist = curr_dist;
        closest_update_rate = up_rate;
      }
    }

    m_update_rate = closest_update_rate;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setUpdateRate()
{
  RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "Xsens Mtw Wrapper... Setting update rate to " << m_update_rate << " Hz"
                                 << BASH_MSG_RESET);


  if (!m_wireless_master_device->setUpdateRate(m_update_rate)) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to set update rate: " << utils::toString(*m_wireless_master_device));

    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setRadioChannel()
{
  if (m_wireless_master_device->isRadioEnabled()) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Disabling previously enabled radio channel");

    if (!disableRadio()) {
      return false;
    }
  }

  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Setting radio channel to " << m_mtw_params.desired_radio_channel << " and enabling radio");
                                                                   
  if (!m_wireless_master_device->enableRadio(m_mtw_params.desired_radio_channel)) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to set radio channel: " << utils::toString(*m_wireless_master_device));

    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::disableRadio()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Disabling radio");

  if (!m_wireless_master_device->disableRadio()) {
    
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to disable radio: " << utils::toString(*m_wireless_master_device));

    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::setupRos()
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Setting up ROS");

  // m_reset_orientation_srv = m_nh.advertiseService("reset_orientation", &Wrapper::resetOrientation, this);
 
  // this->create_service<vi_interfaces::srv::ResetOrientation>("reset_orientation", &&hiros::xsens_mtw::Wrapper::resetOrientation);
                              // std::bind(&hiros::xsens_mtw::Wrapper::resetOrientation, this, std::placeholders::_1, std::placeholders::_2));


  if (m_wrapper_params.publish_mimu_array) {
    // m_data_pub = m_nh.advertise<vi_interfaces::msg::MIMUArray>("data", m_ros_topic_queue_size);
    m_data_pub = this->create_publisher<vi_interfaces::msg::MIMUArray>("data", m_ros_topic_queue_size);

  }
  else {
    for (auto& device : m_connected_devices) {
      if (m_wrapper_params.publish_imu) {
        m_imu_pubs.emplace(
          device.first,
          // m_nh.advertise<sensor_msgs::msg::Imu>(composeTopicPrefix(device.first) + "/imu/data", m_ros_topic_queue_size));
          this->create_publisher<sensor_msgs::msg::Imu>(composeTopicPrefix(device.first) + "/imu/data", m_ros_topic_queue_size));
      }

      if (m_wrapper_params.publish_mag) {
        m_mag_pubs.emplace(device.first,
                          //  m_nh.advertise<sensor_msgs::msg::MagneticField>(composeTopicPrefix(device.first) + "/imu/mag", m_ros_topic_queue_size));
                          this->create_publisher<sensor_msgs::msg::MagneticField>(composeTopicPrefix(device.first) + "/imu/mag", m_ros_topic_queue_size));

      }

      if (m_wrapper_params.publish_euler) {
        m_euler_pubs.emplace(device.first,
                            //  m_nh.advertise<vi_interfaces::msg::Euler>(composeTopicPrefix(device.first) + "/imu/euler", m_ros_topic_queue_size));
                            this->create_publisher<vi_interfaces::msg::Euler>(composeTopicPrefix(device.first) + "/imu/euler", m_ros_topic_queue_size));

      }

      if (m_wrapper_params.publish_free_acceleration) {
        m_free_acceleration_pubs.emplace(
          device.first,
          // m_nh.advertise<geometry_msgs::msg::Vector3Stamped>(composeTopicPrefix(device.first) + "/filter/free_acceleration", m_ros_topic_queue_size));
          this->create_publisher<geometry_msgs::msg::Vector3Stamped>(composeTopicPrefix(device.first) + "/filter/free_acceleration", m_ros_topic_queue_size));

      }

      if (m_wrapper_params.publish_pressure) {
        m_pressure_pubs.emplace(device.first,
                                // m_nh.advertise<sensor_msgs::msg::FluidPressure>(composeTopicPrefix(device.first) + "/pressure", m_ros_topic_queue_size));
                                this->create_publisher<sensor_msgs::msg::FluidPressure>(composeTopicPrefix(device.first) + "/pressure", m_ros_topic_queue_size));

      }
    }
  }
}

void hiros::xsens_mtw::Wrapper::syncInitialPackets()
{
  double delta_t = 1 / static_cast<double>(m_update_rate);
  std::map<XsDeviceId, bool> initial_new_packets_received;
  std::map<XsDeviceId, long> initial_packet_ids;
  std::map<XsDeviceId, double> initial_timestamps;
  for (auto& device : m_connected_devices) {
    initial_new_packets_received.emplace(device.first, false);
    initial_packet_ids.emplace(device.first, -1);
    initial_timestamps.emplace(device.first, std::numeric_limits<double>::max());
  }
  bool got_all_initial_new_packets = false;

  while (!got_all_initial_new_packets) {
    for (auto& device : m_connected_devices) {
      if (!initial_new_packets_received.at(device.first) && m_mtw_callbacks.at(device.first)->newDataAvailable()) {
        std::shared_ptr<XsDataPacket> packet = m_mtw_callbacks.at(device.first)->getLatestPacket();

        if ((packet->timeOfArrival().secTime() - initial_timestamps.at(device.first)) < (0.5 * delta_t)) {
          // First message
          initial_packet_ids.at(device.first) = packet->packetId();
          initial_timestamps.at(device.first) = packet->timeOfArrival().secTime();
        }
        else {
          // First message with different time of arrival
          initial_packet_ids.at(device.first) = packet->packetId();
          initial_timestamps.at(device.first) += delta_t;

          initial_new_packets_received.at(device.first) = true;
        }

        m_mtw_callbacks.at(device.first)->deleteOldestPacket();
      }
    }

    got_all_initial_new_packets =
      std::all_of(initial_new_packets_received.begin(),
                  initial_new_packets_received.end(),
                  [](std::map<XsDeviceId, bool>::const_reference p) { return p.second == true; });
  }

  m_initial_packet_id =
    std::max_element(initial_packet_ids.begin(),
                     initial_packet_ids.end(),
                     [](std::map<XsDeviceId, long>::const_reference p1,
                        std::map<XsDeviceId, long>::const_reference p2) { return p1.second < p2.second; })
      ->second;

  for (auto& device : m_connected_devices) {
    initial_timestamps.at(device.first) += ((m_initial_packet_id - initial_packet_ids.at(device.first)) * delta_t);
  }

  // m_initial_timestamp =
  //   ros::Time(std::min_element(initial_timestamps.begin(),
  //                              initial_timestamps.end(),
  //                              [](std::map<XsDeviceId, double>::const_reference t1,
  //                                 std::map<XsDeviceId, double>::const_reference t2) { return t1.second < t2.second; })
  //               ->second);
  m_initial_timestamp =
    rclcpp::Time(std::min_element(initial_timestamps.begin(),
                               initial_timestamps.end(),
                               [](std::map<XsDeviceId, double>::const_reference t1,
                                  std::map<XsDeviceId, double>::const_reference t2) { return t1.second < t2.second; })
                ->second);
}

bool hiros::xsens_mtw::Wrapper::resetInitialOrientation() const
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Resetting initial orientation");

  bool success = true;

  for (auto& device : m_connected_devices) {
    success = device.second->resetOrientation(XRM_Heading) && success;
  }

  if (!success) {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Failed to reset initial orientation");

  }

  return success;
}

std::string hiros::xsens_mtw::Wrapper::getDeviceLabel(const XsDeviceId& t_id) const
{
  return (m_ids_to_labels.find(t_id) != m_ids_to_labels.end()) ? m_ids_to_labels.at(t_id)
                                                               : t_id.toString().toStdString();
}

XsDeviceId hiros::xsens_mtw::Wrapper::getDeviceId(const std::string t_label) const
{
  return (m_labels_to_ids.find(t_label) != m_labels_to_ids.end()) ? m_labels_to_ids.at(t_label)
                                                                  : utils::toXsDeviceId(t_label);
}

std::string hiros::xsens_mtw::Wrapper::composeTopicPrefix(const XsDeviceId& t_id) const
{
  return "/" + m_node_namespace + "/S" + getDeviceLabel(t_id);
}

void hiros::xsens_mtw::Wrapper::publishPacket(const std::shared_ptr<XsDataPacket>& t_packet)
{
    RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "in publish packet" << BASH_MSG_RESET);


  if (m_wrapper_params.publish_imu && t_packet->containsOrientation() && t_packet->containsCalibratedGyroscopeData()
      && t_packet->containsCalibratedAcceleration()) {
    m_imu_pubs.at(t_packet->deviceId())->publish(getImuMsg(t_packet));
  }

    RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "F1" << BASH_MSG_RESET);


  if (m_wrapper_params.publish_mag && t_packet->containsCalibratedMagneticField()) {
    m_mag_pubs.at(t_packet->deviceId())->publish(getMagMsg(t_packet));
  }

    RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "F2" << BASH_MSG_RESET);


  if (m_wrapper_params.publish_euler && t_packet->containsOrientation()) {
    m_euler_pubs.at(t_packet->deviceId())->publish(getEulerMsg(t_packet));
  }

    RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "F3" << BASH_MSG_RESET);

  if (m_wrapper_params.publish_free_acceleration && t_packet->containsFreeAcceleration()) {
    m_free_acceleration_pubs.at(t_packet->deviceId())->publish(getFreeAccelerationMsg(t_packet));
  }

    RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "F4" << BASH_MSG_RESET);


  if (m_wrapper_params.publish_pressure && t_packet->containsPressure()) {
    m_pressure_pubs.at(t_packet->deviceId())->publish(getPressureMsg(t_packet));
  }

      RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "F5" << BASH_MSG_RESET);


  if (m_wrapper_params.publish_tf && t_packet->containsOrientation()) {
    m_tf_broadcaster->sendTransform(getTf(t_packet));
  }

      RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "end publish pack" << BASH_MSG_RESET);

}

void hiros::xsens_mtw::Wrapper::publishFrame(const std::vector<std::shared_ptr<XsDataPacket>>& t_frame)
{
  if (m_wrapper_params.publish_mimu_array) {
    m_data_pub->publish(getMIMUArrayMsg(t_frame));

    if (m_wrapper_params.publish_tf) {
      for (auto& packet : t_frame) {
        if (packet->containsOrientation()) {
          m_tf_broadcaster->sendTransform(getTf(packet));
        }
      }
    }
  }
  else {
    for (auto& packet : t_frame) {
      publishPacket(packet);
    }
  }
//probably not needed in ros2
  // ros::spinOnce();
}

std_msgs::msg::Header hiros::xsens_mtw::Wrapper::getHeader(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  std_msgs::msg::Header header;

//no seq in ros2
  // header.seq = static_cast<unsigned int>(t_packet->packetId());
  // header.stamp = m_initial_timestamp
                //  + ros::Duration((t_packet->packetId() - m_initial_packet_id) / static_cast<double>(m_update_rate));

  double time_in_seconds = (t_packet->packetId() - m_initial_packet_id) / static_cast<double>(m_update_rate);
  int64_t nanoseconds = static_cast<int64_t>(time_in_seconds * 1e9);
  header.stamp = m_initial_timestamp + rclcpp::Duration::from_nanoseconds(nanoseconds);

  // header.stamp = m_initial_timestamp
                //  + rclcpp::Duration((t_packet->packetId() - m_initial_packet_id) / static_cast<double>(m_update_rate));
                 
  header.frame_id = m_wrapper_params.tf_prefix + getDeviceLabel(t_packet->deviceId());

  return header;
}

sensor_msgs::msg::Imu hiros::xsens_mtw::Wrapper::getImuMsg(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  sensor_msgs::msg::Imu out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.orientation.x = t_packet->orientationQuaternion().x();
  out_msg.orientation.y = t_packet->orientationQuaternion().y();
  out_msg.orientation.z = t_packet->orientationQuaternion().z();
  out_msg.orientation.w = t_packet->orientationQuaternion().w();
  out_msg.orientation_covariance.front() = 0.0;

  out_msg.angular_velocity.x = t_packet->calibratedGyroscopeData().at(0);
  out_msg.angular_velocity.y = t_packet->calibratedGyroscopeData().at(1);
  out_msg.angular_velocity.z = t_packet->calibratedGyroscopeData().at(2);
  out_msg.angular_velocity_covariance.front() = 0.0;

  out_msg.linear_acceleration.x = t_packet->calibratedAcceleration().at(0);
  out_msg.linear_acceleration.y = t_packet->calibratedAcceleration().at(1);
  out_msg.linear_acceleration.z = t_packet->calibratedAcceleration().at(2);
  out_msg.linear_acceleration_covariance.front() = 0.0;

  return out_msg;
}

sensor_msgs::msg::MagneticField hiros::xsens_mtw::Wrapper::getMagMsg(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  sensor_msgs::msg::MagneticField out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.magnetic_field.x = t_packet->calibratedMagneticField().at(0) * 1e-4; // G to T
  out_msg.magnetic_field.y = t_packet->calibratedMagneticField().at(1) * 1e-4; // G to T
  out_msg.magnetic_field.z = t_packet->calibratedMagneticField().at(2) * 1e-4; // G to T
  out_msg.magnetic_field_covariance.front() = 0.0;

  return out_msg;
}

// hiros_xsens_mtw_wrapper::Euler
vi_interfaces::msg::Euler
hiros::xsens_mtw::Wrapper::getEulerMsg(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  vi_interfaces::msg::Euler out_msg;
  out_msg.header = getHeader(t_packet);

  // roll = atan2(2 * (qw * qx + qy * qz), (1 - 2 * (pow(qx, 2) + pow(qy, 2))))
  out_msg.roll = t_packet->orientationEuler().roll();
  // pitch = asin(2 * (qw * qy - qz * qx))
  out_msg.pitch = t_packet->orientationEuler().pitch();
  // yaw = atan2(2 * (qw * qz + qx * qy), (1 - 2 * (pow(qy, 2) + pow(qz, 2))))
  out_msg.yaw = t_packet->orientationEuler().yaw();

  return out_msg;
}

geometry_msgs::msg::Vector3Stamped
hiros::xsens_mtw::Wrapper::getFreeAccelerationMsg(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  geometry_msgs::msg::Vector3Stamped out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.vector.x = t_packet->freeAcceleration().at(0);
  out_msg.vector.y = t_packet->freeAcceleration().at(1);
  out_msg.vector.z = t_packet->freeAcceleration().at(2);

  return out_msg;
}

sensor_msgs::msg::FluidPressure
hiros::xsens_mtw::Wrapper::getPressureMsg(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  sensor_msgs::msg::FluidPressure out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.fluid_pressure = t_packet->pressure().m_pressure;
  out_msg.variance = 0.0;

  return out_msg;
}

// hiros_xsens_mtw_wrapper::MIMU hiros::xsens_mtw::Wrapper::getMIMUMsg(const std::shared_ptr<XsDataPacket>& t_packet) const
vi_interfaces::msg::MIMU hiros::xsens_mtw::Wrapper::getMIMUMsg(const std::shared_ptr<XsDataPacket>& t_packet) const

{
  // hiros_xsens_mtw_wrapper::MIMU out_msg;
  vi_interfaces::msg::MIMU out_msg;


  if (m_wrapper_params.publish_imu && t_packet->containsOrientation() && t_packet->containsCalibratedGyroscopeData()
      && t_packet->containsCalibratedAcceleration()) {
    out_msg.imu = getImuMsg(t_packet);
  }

  if (m_wrapper_params.publish_mag && t_packet->containsCalibratedMagneticField()) {
    out_msg.mag = getMagMsg(t_packet);
  }

  if (m_wrapper_params.publish_euler && t_packet->containsOrientation()) {
    out_msg.euler = getEulerMsg(t_packet);
  }

  if (m_wrapper_params.publish_free_acceleration && t_packet->containsFreeAcceleration()) {
    out_msg.free_acceleration = getFreeAccelerationMsg(t_packet);
  }

  if (m_wrapper_params.publish_pressure && t_packet->containsPressure()) {
    out_msg.pressure = getPressureMsg(t_packet);
  }

  return out_msg;
}

// hiros_xsens_mtw_wrapper::MIMUArray
vi_interfaces::msg::MIMUArray
hiros::xsens_mtw::Wrapper::getMIMUArrayMsg(const std::vector<std::shared_ptr<XsDataPacket>>& t_frame) const
{
  // hiros_xsens_mtw_wrapper::MIMUArray out_msg;
  vi_interfaces::msg::MIMUArray out_msg;


  for (auto& packet : t_frame) {
    out_msg.mimus.push_back(getMIMUMsg(packet));
  }

  return out_msg;
}

geometry_msgs::msg::TransformStamped hiros::xsens_mtw::Wrapper::getTf(const std::shared_ptr<XsDataPacket>& t_packet) const
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header = getHeader(t_packet);
  tf.header.frame_id = "world";
  tf.child_frame_id = m_wrapper_params.tf_prefix + getDeviceLabel(t_packet->deviceId());

  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = t_packet->orientationQuaternion().x();
  tf.transform.rotation.y = t_packet->orientationQuaternion().y();
  tf.transform.rotation.z = t_packet->orientationQuaternion().z();
  tf.transform.rotation.w = t_packet->orientationQuaternion().w();

  return tf;
}

// bool hiros::xsens_mtw::Wrapper::resetOrientation(hiros_xsens_mtw_wrapper::ResetOrientation::Request& t_req,
                                                //  hiros_xsens_mtw_wrapper::ResetOrientation::Response& t_res)
bool hiros::xsens_mtw::Wrapper::resetOrientation(vi_interfaces::srv::ResetOrientation::Request& t_req,
                                                 vi_interfaces::srv::ResetOrientation::Response& t_res)
{
  bool success = true;

  if (t_req.sensors.empty()) {
    RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "Xsens Mtw Wrapper... Resetting orientation of all connected sensors"
                                   << BASH_MSG_RESET);

    for (auto& device : m_connected_devices) {
      success = device.second->resetOrientation(XRM_Alignment) && success;
    }
  }
  else {
    for (auto& sensor_label : t_req.sensors) {
      if (m_connected_devices.find(getDeviceId(sensor_label)) != m_connected_devices.end()) {
        RCLCPP_INFO_STREAM(this->get_logger(), BASH_MSG_GREEN << "Xsens Mtw Wrapper... Resetting orientation of '" << sensor_label << "'"
                                       << BASH_MSG_RESET);

        success = m_connected_devices.at(getDeviceId(sensor_label))->resetOrientation(XRM_Alignment) && success;
      }
      else {
        RCLCPP_WARN_STREAM(this->get_logger(), "Xsens Mtw Wrapper... Cannot find '" << sensor_label << "'");

        success = false;
      }
    }
  }

  return success;
}
