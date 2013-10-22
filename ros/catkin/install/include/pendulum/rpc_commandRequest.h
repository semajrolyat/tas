/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/james/tas/ros/catkin/src/pendulum/srv/rpc_command.srv
 *
 */


#ifndef PENDULUM_MESSAGE_RPC_COMMANDREQUEST_H
#define PENDULUM_MESSAGE_RPC_COMMANDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pendulum
{
template <class ContainerAllocator>
struct rpc_commandRequest_
{
  typedef rpc_commandRequest_<ContainerAllocator> Type;

  rpc_commandRequest_()
    : time(0.0)
    , position(0.0)
    , velocity(0.0)  {
    }
  rpc_commandRequest_(const ContainerAllocator& _alloc)
    : time(0.0)
    , position(0.0)
    , velocity(0.0)  {
    }



   typedef double _time_type;
  _time_type time;

   typedef double _position_type;
  _position_type position;

   typedef double _velocity_type;
  _velocity_type velocity;




  typedef boost::shared_ptr< ::pendulum::rpc_commandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pendulum::rpc_commandRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct rpc_commandRequest_

typedef ::pendulum::rpc_commandRequest_<std::allocator<void> > rpc_commandRequest;

typedef boost::shared_ptr< ::pendulum::rpc_commandRequest > rpc_commandRequestPtr;
typedef boost::shared_ptr< ::pendulum::rpc_commandRequest const> rpc_commandRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pendulum::rpc_commandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pendulum::rpc_commandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pendulum

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg'], 'pendulum': ['/home/james/tas/ros/catkin/src/pendulum/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pendulum::rpc_commandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pendulum::rpc_commandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pendulum::rpc_commandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3601c7c34e65d3f14462f3a362199d0c";
  }

  static const char* value(const ::pendulum::rpc_commandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3601c7c34e65d3f1ULL;
  static const uint64_t static_value2 = 0x4462f3a362199d0cULL;
};

template<class ContainerAllocator>
struct DataType< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pendulum/rpc_commandRequest";
  }

  static const char* value(const ::pendulum::rpc_commandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 time\n\
float64 position\n\
float64 velocity\n\
\n\
";
  }

  static const char* value(const ::pendulum::rpc_commandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.position);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct rpc_commandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pendulum::rpc_commandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pendulum::rpc_commandRequest_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "position: ";
    Printer<double>::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PENDULUM_MESSAGE_RPC_COMMANDREQUEST_H
