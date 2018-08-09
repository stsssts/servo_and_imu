#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

extern "C"
{
#include <rc_usefulincludes.h>
#include <roboticscape.h>
}

class PWMChannel
{
public:
  PWMChannel(ros::NodeHandle nh, int channel_id)
  {
    _channel_id = channel_id;
    std::string id = std::to_string(channel_id);

    int min_pulse_width;
    int max_pulse_width;
    float min_scale_value;
    float max_scale_value;
    std::string topic;

    ros::NodeHandle pnh("~");
    pnh.param("ch" + id + "_pulse_width", _pulse_width, int(1500));
    pnh.param("ch" + id + "_min_pulse_width", min_pulse_width, int(1100));
    pnh.param("ch" + id + "_max_pulse_width", max_pulse_width, int(1900));
    pnh.param("ch" + id + "_min_value", min_scale_value, float(1100));
    pnh.param("ch" + id + "_max_value", max_scale_value, float(1900));
    pnh.param("ch" + id + "_topic", topic, "servo_" + id);

    scale_k = (max_pulse_width - min_pulse_width) / (max_scale_value - min_scale_value);
    scale_shift = (max_scale_value * min_pulse_width - min_scale_value * max_pulse_width) /
        (max_scale_value - min_scale_value);

    _subscriber = nh.subscribe(topic + std::string("/pulse_width"), 10,
                                &PWMChannel::callback, this);
    ROS_INFO("Servo %i initialised", channel_id);
  }

  void update()
  {
    if (_pulse_width_received != _pulse_width)
      _pulse_width = _pulse_width_received;

    rc_send_servo_pulse_us(_channel_id, _pulse_width);
  }

  int scaleLinear(float data)
  {
    return int(data*scale_k + scale_shift);
  }

  void callback(const std_msgs::Float32 msg)
  {
    _pulse_width_received = scaleLinear(msg.data);
  }

private:
  ros::Subscriber _subscriber;

  int _channel_id;
  int _pulse_width;
  int _pulse_width_received;

  float scale_k;
  float scale_shift;
};

class Imu
{
public:
  Imu(ros::NodeHandle nh) : _node("~")
  {
    // start with default config and modify based on options
    rc_imu_config_t conf = rc_default_imu_config();

    // now set up the imu for dmp interrupt operation
    if(rc_initialize_imu_dmp(&_data, conf))
    {
      ROS_FATAL("rc_imu_initialize_failed");
      return;
    }

    ROS_INFO("Imu initialized");
    _imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
  }

  void publishImu()
  {
    sensor_msgs::Imu msg;
    msg.header.frame_id = "robot";

    msg.orientation.w = _data.dmp_quat[QUAT_W];
    msg.orientation.x = _data.dmp_quat[QUAT_X];
    msg.orientation.y = _data.dmp_quat[QUAT_Y];
    msg.orientation.z = _data.dmp_quat[QUAT_Z];

    msg.angular_velocity.x = _data.gyro[0];
    msg.angular_velocity.y = _data.gyro[1];
    msg.angular_velocity.z = _data.gyro[2];

    _imu_publisher.publish(msg);
  }

private:
  rc_imu_data_t _data;
  ros::NodeHandle _node;
  ros::Publisher _imu_publisher;
};

/**
 * @brief      interrupt handler to catch ctrl-c
 */
void signal_handler(__attribute__ ((unused)) int dummy)
{
  ros::shutdown();
  return;
}

int main(int argc, char **argv)
{
  signal(SIGINT, signal_handler);
  ros::init(argc, argv, "servo_and_imu_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  int frequency, channels_len;
  pnh.param("frequency", frequency, 50);
  pnh.param("channels", channels_len, 8);

  std::vector<PWMChannel> channels;

  for (int i = 0; i < channels_len; ++i)
    channels.push_back(PWMChannel(nh, i+1));

  if(rc_initialize()<0)
  {
    ROS_ERROR("Failed to initialize cape.");
    return EXIT_FAILURE;
  }

  Imu imu(nh);

  rc_set_led(RED,1);
  rc_set_led(GREEN,0);
  rc_set_state(UNINITIALIZED);

  if(rc_enable_servo_power_rail()){
    fprintf(stderr,"failed to enable power rail\n");
    return -1;
  }

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    for (auto ch : channels)
      ch.update();
    imu.publishImu();

    ros::spinOnce();
    loop_rate.sleep();
  }

  rc_set_state(EXITING);
  rc_power_off_imu();
  rc_disable_servo_power_rail();
  rc_cleanup();

  return EXIT_SUCCESS;
}
