#ifndef _ROS_gripper_driver_motor_command_h
#define _ROS_gripper_driver_motor_command_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gripper_driver
{

  class motor_command : public ros::Msg
  {
    public:
      typedef bool _read_angle_type;
      _read_angle_type read_angle;
      typedef bool _read_load_type;
      _read_load_type read_load;
      typedef bool _gripper_ready_type;
      _gripper_ready_type gripper_ready;
      typedef bool _gripper_open_type;
      _gripper_open_type gripper_open;
      typedef bool _gripper_close_type;
      _gripper_close_type gripper_close;
      typedef bool _gripper_standby_type;
      _gripper_standby_type gripper_standby;

    motor_command():
      read_angle(0),
      read_load(0),
      gripper_ready(0),
      gripper_open(0),
      gripper_close(0),
      gripper_standby(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_read_angle;
      u_read_angle.real = this->read_angle;
      *(outbuffer + offset + 0) = (u_read_angle.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->read_angle);
      union {
        bool real;
        uint8_t base;
      } u_read_load;
      u_read_load.real = this->read_load;
      *(outbuffer + offset + 0) = (u_read_load.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->read_load);
      union {
        bool real;
        uint8_t base;
      } u_gripper_ready;
      u_gripper_ready.real = this->gripper_ready;
      *(outbuffer + offset + 0) = (u_gripper_ready.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_ready);
      union {
        bool real;
        uint8_t base;
      } u_gripper_open;
      u_gripper_open.real = this->gripper_open;
      *(outbuffer + offset + 0) = (u_gripper_open.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_open);
      union {
        bool real;
        uint8_t base;
      } u_gripper_close;
      u_gripper_close.real = this->gripper_close;
      *(outbuffer + offset + 0) = (u_gripper_close.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_close);
      union {
        bool real;
        uint8_t base;
      } u_gripper_standby;
      u_gripper_standby.real = this->gripper_standby;
      *(outbuffer + offset + 0) = (u_gripper_standby.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripper_standby);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_read_angle;
      u_read_angle.base = 0;
      u_read_angle.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->read_angle = u_read_angle.real;
      offset += sizeof(this->read_angle);
      union {
        bool real;
        uint8_t base;
      } u_read_load;
      u_read_load.base = 0;
      u_read_load.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->read_load = u_read_load.real;
      offset += sizeof(this->read_load);
      union {
        bool real;
        uint8_t base;
      } u_gripper_ready;
      u_gripper_ready.base = 0;
      u_gripper_ready.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_ready = u_gripper_ready.real;
      offset += sizeof(this->gripper_ready);
      union {
        bool real;
        uint8_t base;
      } u_gripper_open;
      u_gripper_open.base = 0;
      u_gripper_open.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_open = u_gripper_open.real;
      offset += sizeof(this->gripper_open);
      union {
        bool real;
        uint8_t base;
      } u_gripper_close;
      u_gripper_close.base = 0;
      u_gripper_close.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_close = u_gripper_close.real;
      offset += sizeof(this->gripper_close);
      union {
        bool real;
        uint8_t base;
      } u_gripper_standby;
      u_gripper_standby.base = 0;
      u_gripper_standby.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->gripper_standby = u_gripper_standby.real;
      offset += sizeof(this->gripper_standby);
     return offset;
    }

    const char * getType(){ return "gripper_driver/motor_command"; };
    const char * getMD5(){ return "8abb8dc4e270785b228e26d3dff0970e"; };

  };

}
#endif