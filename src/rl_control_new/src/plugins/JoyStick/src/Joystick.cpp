#include "../include/Joystick.h"

#ifdef USE_ROS_JOY
Joystick_humanoid::Joystick_humanoid()
{
  std::cout << "Joystick Start" << std::endl;
  // sub_ = nh_.subscribe<sensor_msgs::Joy>("/sbus_data", 1000, &Joystick_humanoid::xbox_map_read, this);
}

Joystick_humanoid::~Joystick_humanoid()
{
  std::cout << "Joystick End" << std::endl;
}

void Joystick_humanoid::xbox_map_read(const sensor_msgs::Joy::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(data_mutex);
  xbox_map_.a = msg->axes[8];
  xbox_map_.b = msg->axes[9];
  xbox_map_.c = msg->axes[10];
  xbox_map_.d = msg->axes[11];
  xbox_map_.e = msg->axes[4];
  xbox_map_.f = msg->axes[7];
  xbox_map_.g = msg->axes[5];
  xbox_map_.h = msg->axes[6];
  xbox_map_.x1 = msg->axes[3];
  xbox_map_.x2 = msg->axes[0];
  xbox_map_.y1 = msg->axes[2];
  xbox_map_.y2 = msg->axes[1];
}

void Joystick_humanoid::xbox_flag_update(xbox_map_t xbox_map)
{
  xbox_map_ = xbox_map;
  std::lock_guard<std::mutex> lock(data_mutex);
  // Joint enable flag
  if (xbox_map_.e == 1.0 && xbox_map_.b == 1.0)
  {
    xbox_flag_.is_disable = true;
  }
  else
  {
    xbox_flag_.is_disable = false;
  }
  // Get mlp state change flag
  xbox_flag_.fsm_state_command = (xbox_map_.d == 1.0)   ? "gotoZero"
                                 : (xbox_map_.c == 1.0) ? "gotoStop"
                                 : (xbox_map_.a == 1.0) ? "gotoMLP"
                                                        : xbox_flag_.fsm_state_command;

  // Get command gait
  xbox_flag_.command_gait = (xbox_map_.a == 1.0) ? 0.0 : (xbox_map_.f == 1.0) ? 1.0
                                                      : (xbox_map_.h == -1.0)  ? 2.0
                                                      : (xbox_map_.h == 1.0)   ? 3.0
                                                                               : xbox_flag_.command_gait;
  // Get speed
  xbox_flag_.y_speed_command = xbox_map_.x1 * -0.25; 
  if (xbox_map_.y1 > 0){
    xbox_flag_.x_speed_command = xbox_map_.y1 * 1.5;
  } 
  else{
    xbox_flag_.x_speed_command = xbox_map_.y1 * 0.25;
  }
  
  xbox_flag_.yaw_speed_command = xbox_map_.x2 * -0.8;
}

xbox_flag Joystick_humanoid::get_xbox_flag()
{
  return xbox_flag_;
}

int Joystick_humanoid::init()
{
  return 0;
}
#else
#include <iostream>

Joystick_humanoid::Joystick_humanoid()
{
  exit_flag_.store(false);
}

Joystick_humanoid::~Joystick_humanoid()
{
  exit_flag_.store(true);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "joystick end" << std::endl;
}

int Joystick_humanoid::xbox_open(const char *file_name)
{
  int xbox_fd;

  xbox_fd = open(file_name, O_RDONLY);
  if (xbox_fd < 0)
  {
    perror("open");
    return -1;
  }

  return xbox_fd;
}

int Joystick_humanoid::xbox_map_read(xbox_map_t *map) const
{
  int len, type, number, value;
  struct js_event js;

  len = read(xbox_fd, &js, sizeof(struct js_event));
  if (len < 0)
  {
    perror("read");
    return -1;
  }

  type = js.type;
  number = js.number;
  value = js.value;

  map->time = js.time;

  if (type == JS_EVENT_BUTTON)
  {
    switch (number)
    {
    case XBOX_BUTTON_A:
      map->a = value;
      break;

    case XBOX_BUTTON_B:
      map->b = value;
      break;

    case XBOX_BUTTON_X:
      map->x = value;
      break;

    case XBOX_BUTTON_Y:
      map->y = value;
      break;

    case XBOX_BUTTON_LB:
      map->lb = value;
      break;

    case XBOX_BUTTON_RB:
      map->rb = value;
      break;

    case XBOX_BUTTON_START:
      map->start = value;
      break;

    case XBOX_BUTTON_BACK:
      map->back = value;
      break;

    case XBOX_BUTTON_HOME:
      map->home = value;
      break;

    case XBOX_BUTTON_LO:
      map->lo = value;
      break;

    case XBOX_BUTTON_RO:
      map->ro = value;
      break;

    default:
      break;
    }
  }
  else if (type == JS_EVENT_AXIS)
  {
    switch (number)
    {
    case XBOX_AXIS_LX:
      map->lx = value;
      break;

    case XBOX_AXIS_LY:
      map->ly = value;
      break;

    case XBOX_AXIS_RX:
      map->rx = value;
      break;

    case XBOX_AXIS_RY:
      map->ry = value;
      break;

    case XBOX_AXIS_LT:
      map->lt = value;
      break;

    case XBOX_AXIS_RT:
      map->rt = value;
      break;

    case XBOX_AXIS_XX:
      map->xx = value;
      break;

    case XBOX_AXIS_YY:
      map->yy = value;
      break;

    default:
      break;
    }
  }
  else
  {
    /* Init do nothing */
  }

  return len;
}

void Joystick_humanoid::xbox_close() const
{
  close(xbox_fd);
}

int Joystick_humanoid::xbox_init()
{
  xbox_fd = xbox_open("/dev/input/js0");
  if (xbox_fd < 0)
  {
    return -1;
  }
  return 0;
}

void Joystick_humanoid::update_xbox_flag(xbox_map_t *map)
{
  std::lock_guard<std::mutex> lock(flag_mutex_);
  // Joint enable flag
  xbox_flag_.is_disable = (map->lt > 1000 && map->rt > 1000);
  // Get state change flag
  xbox_flag_.fsm_state_command = (map->a == 1.0) ? "gotoZero" : (map->b == 1.0) ? "gotoStop"
                                                            : (map->x == 1.0)   ? "gotoMLP"
                                                                                : xbox_flag_.fsm_state_command;
  // Get command gait
  xbox_flag_.command_gait = (map->x == 1.0) ? 0.0 : (map->y == 1.0) ? 1.0
                                                : (map->rb == 1.0)  ? 2.0
                                                : (map->lb == 1.0)  ? 3.0
                                                                    : xbox_flag_.command_gait;
  // Get x speed
  if (map->lt < 1000)
  {
    if ((abs(map->ly) > dead_area) && (abs(map->ly) <= max_value))
    {
      if (map->ly > 0)
      {
        xbox_flag_.x_speed_command = ly_dir * max_speed_x * ((double)(abs(map->ly) - dead_area) / (max_value - dead_area));
      }
      else
      {
        xbox_flag_.x_speed_command = ly_dir * min_speed_x * ((double)(abs(map->ly) - dead_area) / (max_value - dead_area));
      }
    }
    else
    {
      xbox_flag_.x_speed_command = 0.0;
    }
  }
  else
  {
    xbox_flag_.x_speed_command = 0.25;
  }
  // Get y speed
  if ((abs(map->lx) > dead_area) && (abs(map->lx) <= max_value))
  {
    if (map->lx > 0)
    {
      xbox_flag_.y_speed_command = lx_dir * max_speed_y * ((double)(abs(map->lx) - dead_area) / (max_value - dead_area));
    }
    else
    {
      xbox_flag_.y_speed_command = lx_dir * min_speed_y * ((double)(abs(map->lx) - dead_area) / (max_value - dead_area));
    }
  }
  else
  {
    xbox_flag_.y_speed_command = 0.0;
  }
  // Get yaw speed
  if ((abs(map->rx) > dead_area) && (abs(map->rx) <= max_value))
  {
    xbox_flag_.yaw_speed_command = (map->rx > 0) ? rx_dir * max_speed_yaw * ((double)(abs(map->rx) - dead_area) / (max_value - dead_area)) : rx_dir * min_speed_yaw * ((double)(abs(map->rx) - dead_area) / (max_value - dead_area));
  }
  else
  {
    xbox_flag_.yaw_speed_command = 0.0;
  }
  // Get x offset
  if ((map->yy < -30000) && (abs(last_value_x) < 500))
  {
    last_value_x = map->yy;
    xbox_flag_.x_speed_offset = delta_offset_x;
  }
  else if ((map->yy > 30000) && (abs(last_value_x) < 500))
  {
    last_value_x = map->yy;
    xbox_flag_.x_speed_offset = -delta_offset_x;
  }
  else
  {
    last_value_x = map->yy;
    xbox_flag_.x_speed_offset = 0.0;
  }
  // Get y offset
  if ((map->xx < -30000) && (abs(last_value_y) < 500))
  {
    last_value_y = map->xx;
    xbox_flag_.y_speed_offset = delta_offset_y;
  }
  else if ((map->xx > 30000) && (abs(last_value_y) < 500))
  {
    last_value_y = map->xx;
    xbox_flag_.y_speed_offset = -delta_offset_y;
  }
  else
  {
    last_value_y = map->xx;
    xbox_flag_.y_speed_offset = 0.0;
  }
  // Get motion number
  int motion_add_number = 0;

  if (map->xx < -30000 && last_value_xx == 0)
  {
    motion_add_number = -1;
  }
  else if (map->xx > 30000 && last_value_xx == 0)
  {
    motion_add_number = 1;
  }
  last_value_xx = map->xx;
  xbox_flag_.motion_number = motion_add_number;
  // Get motion state
  xbox_flag_.motion_state = (map->rt < 1000) ? false : true;
}

xbox_flag Joystick_humanoid::get_xbox_flag()
{
  std::lock_guard<std::mutex> lock(flag_mutex_);
  return xbox_flag_;
}

int Joystick_humanoid::xbox_read(xbox_map_t *xbox_m)
{

  int len = xbox_map_read(xbox_m);
  if (len < 0)
  {
    return -1;
  }
  return 0;
}

void Joystick_humanoid::init()
{
  // int ret = -1;
  xbox_m_.a = 0.0;
  xbox_m_.b = 0.0;
  xbox_m_.x = 0.0;
  xbox_m_.y = 0.0;
  xbox_m_.lx = 0.0;
  xbox_m_.ly = 0.0;
  xbox_m_.rx = 0.0;
  xbox_m_.ry = 0.0;
  xbox_m_.xx = 0.0;
  xbox_m_.yy = 0.0;
  xbox_m_.lb = 0.0;
  xbox_m_.rb = 0.0;
  xbox_m_.lt = -32767;
  xbox_m_.rt = -32767;
  xbox_flag_.fsm_state_command = "gotoStop";
  xbox_thread = std::thread(&Joystick_humanoid::xbox_run, this);
  xbox_thread.detach();
}

void Joystick_humanoid::xbox_run()
{
  int len, ret;

  while (1)
  {
    ret = xbox_init();
    if (ret < 0)
    {
      printf("xbox init fail\n");
      usleep(1 * 1000 * 1000);
    }
    else
    {
      std::cout << "xbox connect!" << std::endl;
    }
    while (ret == 0)
    {
      len = xbox_read(&xbox_m_);
      update_xbox_flag(&xbox_m_);
      if (len < 0)
      {
        std::cout << "xbox disconnect!" << std::endl;
        break;
      }

      usleep(10 * 1000);
    }
    if (exit_flag_.load())
    {
      break;
    }
  }
  std::cout << "xbox run end" << std::endl;
}
#endif
