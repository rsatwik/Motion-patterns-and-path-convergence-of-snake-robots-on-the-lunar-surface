// Generated by gencpp from file snake_control/GetJointStatesResponse.msg
// DO NOT EDIT!


#ifndef SNAKE_CONTROL_MESSAGE_GETJOINTSTATESRESPONSE_H
#define SNAKE_CONTROL_MESSAGE_GETJOINTSTATESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace snake_control
{
template <class ContainerAllocator>
struct GetJointStatesResponse_
{
  typedef GetJointStatesResponse_<ContainerAllocator> Type;

  GetJointStatesResponse_()
    : header()
    , position()
    , effort()  {
    }
  GetJointStatesResponse_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , position(_alloc)
    , effort(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _position_type;
  _position_type position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _effort_type;
  _effort_type effort;





  typedef boost::shared_ptr< ::snake_control::GetJointStatesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::snake_control::GetJointStatesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetJointStatesResponse_

typedef ::snake_control::GetJointStatesResponse_<std::allocator<void> > GetJointStatesResponse;

typedef boost::shared_ptr< ::snake_control::GetJointStatesResponse > GetJointStatesResponsePtr;
typedef boost::shared_ptr< ::snake_control::GetJointStatesResponse const> GetJointStatesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::snake_control::GetJointStatesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace snake_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::snake_control::GetJointStatesResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::snake_control::GetJointStatesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::snake_control::GetJointStatesResponse_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bc3247c2ebdce0b3b92a25e3910af17e";
  }

  static const char* value(const ::snake_control::GetJointStatesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbc3247c2ebdce0b3ULL;
  static const uint64_t static_value2 = 0xb92a25e3910af17eULL;
};

template<class ContainerAllocator>
struct DataType< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "snake_control/GetJointStatesResponse";
  }

  static const char* value(const ::snake_control::GetJointStatesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
float64[] position\n\
\n\
float64[] effort\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::snake_control::GetJointStatesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.position);
      stream.next(m.effort);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetJointStatesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::snake_control::GetJointStatesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::snake_control::GetJointStatesResponse_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "position[]" << std::endl;
    for (size_t i = 0; i < v.position.size(); ++i)
    {
      s << indent << "  position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.position[i]);
    }
    s << indent << "effort[]" << std::endl;
    for (size_t i = 0; i < v.effort.size(); ++i)
    {
      s << indent << "  effort[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.effort[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SNAKE_CONTROL_MESSAGE_GETJOINTSTATESRESPONSE_H
