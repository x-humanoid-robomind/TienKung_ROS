#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#define USE_ROS_JOY

#ifdef USE_ROS_JOY

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

typedef struct xbox_map
{
  double a;
  double b;
  double c;
  double d;
  double e;
  double f;
  double g;
  double h;
  double x1;
  double x2;
  double y1;
  double y2;
} xbox_map_t;

struct xbox_flag
{
  bool is_disable;
  std::string fsm_state_command;
  double command_gait = 0.0;
  double x_speed_command = 0.0;
  double y_speed_command = 0.0;
  double yaw_speed_command = 0.0;
  double x_speed_offset = 0.0;
  double y_speed_offset = 0.0;
  int motion_number;
  bool motion_state;
};

class Joystick_humanoid
{
public:
  Joystick_humanoid();
  ~Joystick_humanoid();
  int init();
  xbox_flag get_xbox_flag();
  void xbox_flag_update(xbox_map_t xbox_map);
  void xbox_map_read(const sensor_msgs::Joy::ConstPtr &msg);


private:
  std::mutex data_mutex;
  // ros::NodeHandle nh_;
  // ros::Subscriber sub_;
  xbox_map xbox_map_;
  xbox_flag xbox_flag_;
  // void xbox_map_read(const sensor_msgs::Joy::ConstPtr &msg);
};
#else
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <atomic>
#include <mutex>

#include <linux/input.h>
#include <linux/joystick.h>

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02

// receiver map
// #define XBOX_BUTTON_A       0x00
// #define XBOX_BUTTON_B       0x01
// #define XBOX_BUTTON_X       0x02
// #define XBOX_BUTTON_Y       0x03
// #define XBOX_BUTTON_LB      0x04
// #define XBOX_BUTTON_RB      0x05
// #define XBOX_BUTTON_START   0x06
// #define XBOX_BUTTON_BACK    0x07
// #define XBOX_BUTTON_HOME    0x08
// #define XBOX_BUTTON_LO      0x09    /* 左摇杆按键 */
// #define XBOX_BUTTON_RO      0x0a    /* 右摇杆按键 */

// #define XBOX_BUTTON_ON      0x01
// #define XBOX_BUTTON_OFF     0x00

// #define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */
// #define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */
// #define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */
// #define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */
// #define XBOX_AXIS_LT        0x02
// #define XBOX_AXIS_RT        0x05
// #define XBOX_AXIS_XX        0x06    /* 方向键X轴 */
// #define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */

// bluetooth map
#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x03
#define XBOX_BUTTON_Y 0x04
#define XBOX_BUTTON_LB 0x06
#define XBOX_BUTTON_RB 0x07
#define XBOX_BUTTON_START 0x02
#define XBOX_BUTTON_BACK 0x05
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x02 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x03 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x05
#define XBOX_AXIS_RT 0x04
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

typedef struct xbox_map
{
  int time;
  int a;
  int b;
  int x;
  int y;
  int lb;
  int rb;
  int start;
  int back;
  int home;
  int lo;
  int ro;

  int lx;
  int ly;
  int rx;
  int ry;
  int lt;
  int rt;
  int xx;
  int yy;

} xbox_map_t;

struct xbox_flag
{
  bool is_disable;
  std::string fsm_state_command;
  double command_gait = 0.0;
  double x_speed_command = 0.0;
  double y_speed_command = 0.0;
  double yaw_speed_command = 0.0;
  double x_speed_offset = 0.0;
  double y_speed_offset = 0.0;
  int motion_number;
  bool motion_state;
};

class Joystick_humanoid
{
public:
  Joystick_humanoid();
  ~Joystick_humanoid();
  void init();
  void xbox_run();
  xbox_flag get_xbox_flag();

private:
  // thread safety
  std::mutex flag_mutex_; // 互斥锁
  // fsm state
  xbox_map_t xbox_m_{};
  std::thread xbox_thread;
  std::atomic<bool> exit_flag_;
  // xbox
  xbox_flag xbox_flag_;
  void update_xbox_flag(xbox_map_t *map);
  int xbox_fd{};
  static int xbox_open(const char *file_name);
  int xbox_map_read(xbox_map_t *map) const;
  void xbox_close() const;
  int xbox_init();
  int xbox_read(xbox_map_t *xbox_m);

  int dead_area = 5000;
  int max_value = 32767;
  double lx_dir = -1.0;
  double ly_dir = -1.0;

  double rx_dir = -1.0;
  double ry_dir = -1.0;

  double max_speed_x = 0.25; // -x
  double min_speed_x = -1.0; // +x

  double max_speed_y = 0.25;
  double min_speed_y = -0.25;

  double max_speed_yaw = 0.4;
  double min_speed_yaw = -0.4;

  double delta_offset_x = 0.03;
  double last_value_x = 0;

  double delta_offset_y = 0.03;
  double last_value_y = 0;

  int last_value_xx = 0;
};
#endif

#endif
