#ifndef _ROS_gripper_driver_motor_state_h
#define _ROS_gripper_driver_motor_state_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gripper_driver
{

  class motor_state : public ros::Msg
  {
    public:
      typedef float _angle_type;
      _angle_type angle;
      typedef float _load_type;
      _load_type load;

    motor_state():
      angle(0),
      load(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        float real;
        uint32_t base;
      } u_load;
      u_load.real = this->load;
      *(outbuffer + offset + 0) = (u_load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_load.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_load.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_load.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->load);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        float real;
        uint32_t base;
      } u_load;
      u_load.base = 0;
      u_load.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_load.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_load.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_load.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->load = u_load.real;
      offset += sizeof(this->load);
     return offset;
    }

    const char * getType(){ return "gripper_driver/motor_state"; };
    const char * getMD5(){ return "821acdedeefedb4e32bbd235e52cfd8f"; };

  };

}
#endif