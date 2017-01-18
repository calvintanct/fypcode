#ifndef _ROS_gripper_driver_force_h
#define _ROS_gripper_driver_force_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gripper_driver
{

  class force : public ros::Msg
  {
    public:
      typedef float _force1_type;
      _force1_type force1;
      typedef float _force2_type;
      _force2_type force2;

    force():
      force1(0),
      force2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_force1;
      u_force1.real = this->force1;
      *(outbuffer + offset + 0) = (u_force1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force1);
      union {
        float real;
        uint32_t base;
      } u_force2;
      u_force2.real = this->force2;
      *(outbuffer + offset + 0) = (u_force2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_force1;
      u_force1.base = 0;
      u_force1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force1 = u_force1.real;
      offset += sizeof(this->force1);
      union {
        float real;
        uint32_t base;
      } u_force2;
      u_force2.base = 0;
      u_force2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force2 = u_force2.real;
      offset += sizeof(this->force2);
     return offset;
    }

    const char * getType(){ return "gripper_driver/force"; };
    const char * getMD5(){ return "d9d3e01eac811d943f8a2b6fd30fe322"; };

  };

}
#endif