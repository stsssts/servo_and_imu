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
  PWMChannel(ros::NodeHandle& nh, ros::NodeHandle& pnh, int channel_id)
    : _channel_id(channel_id)
  {
    std::string topic;
    int min_pulse_width;
    int max_pulse_width;
    float min_scale_value;
    float max_scale_value;

    std::string id_str = std::to_string(channel_id);
    pnh.param("ch" + id_str + "_pulse_width", _pulse_width, int(1500));
    pnh.param("ch" + id_str + "_min_pulse_width", min_pulse_width, int(1100));
    pnh.param("ch" + id_str + "_max_pulse_width", max_pulse_width, int(1900));
    pnh.param("ch" + id_str + "_min_value", min_scale_value, float(1100));
    pnh.param("ch" + id_str + "_max_value", max_scale_value, float(1900));
    pnh.param("ch" + id_str + "_topic", topic, "servo_" + id_str);

    scale_k = (max_pulse_width - min_pulse_width) / (max_scale_value - min_scale_value);
    scale_shift = (max_scale_value * min_pulse_width - min_scale_value * max_pulse_width) /
        (max_scale_value - min_scale_value);

    _subscriber = nh.subscribe(topic + std::string("/pulse_width"), 10,
                               &PWMChannel::callback, this);

    ROS_INFO("Channel %i initialized", channel_id);
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

  void callback(const std_msgs::Float32ConstPtr& msg)
  {
    _pulse_width_received = scaleLinear(msg->data);
  }

private:
  ros::Subscriber _subscriber;

  const int _channel_id;
  int _pulse_width;
  int _pulse_width_received;

  float scale_k;
  float scale_shift;
};

class Servo
{
public:
  Servo(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : _initialized(false)
  {
    if(rc_enable_servo_power_rail())
    {
      ROS_ERROR("Failed to initialize servo power rail");
      return;
    }

    int frequency;
    std::vector<int> ch;

    pnh.param("frequency", frequency, 50);
    pnh.param< std::vector<int> >("channels", ch, {1, 2, 3, 4, 5, 6, 7, 8});

    for (auto ch_num : ch)
      _channels.push_back(PWMChannel(nh, pnh, ch_num));

    _initialized = true;
    ROS_INFO("Servo initialization complete");
  }

  void update()
  {
    for (auto ch : _channels)
      ch.update();
  }

  bool ok() { return _initialized; }

private:
  std::vector<PWMChannel> _channels;
  int _frequency;

  bool _initialized;
};

class Imu
{
public:
  Imu(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : _initialized(false)
  {
    if(rc_initialize_imu_dmp(&_data, rc_default_imu_config()))
    {
      ROS_INFO("Failed to initialize IMU");
      return;
    }

    _initialized = true;
    _imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
    _msg.header.frame_id = "robot";

    ROS_INFO("Imu initialized");
  }

  ~Imu()
  {
    if (_initialized)
      rc_power_off_imu();
  }

  void update()
  {
    _msg.orientation.w = _data.dmp_quat[QUAT_W];
    _msg.orientation.x = _data.dmp_quat[QUAT_X];
    _msg.orientation.y = _data.dmp_quat[QUAT_Y];
    _msg.orientation.z = _data.dmp_quat[QUAT_Z];

    _msg.angular_velocity.x = _data.gyro[0];
    _msg.angular_velocity.y = _data.gyro[1];
    _msg.angular_velocity.z = _data.gyro[2];

    _imu_publisher.publish(_msg);
  }

  bool ok() { return _initialized; }

private:
  rc_imu_data_t _data;
  ros::Publisher _imu_publisher;
  sensor_msgs::Imu _msg;

  bool _initialized;
};

void signal_handler(__attribute__ ((unused)) int dummy)
{
  rc_set_state(EXITING);
  rc_cleanup();
  ROS_WARN("my signal handler");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_and_imu_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  if(rc_initialize())
  {
    ROS_ERROR("Failed to initialize cape");
    return EXIT_FAILURE;
  }

  Imu imu(nh, pnh);
  Servo servo(nh, pnh);
  if (!imu.ok() || !servo.ok())
    return EXIT_FAILURE;

  signal(SIGINT, signal_handler);
  rc_set_state(RUNNING);

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    imu.update();
    servo.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return EXIT_SUCCESS;
}
