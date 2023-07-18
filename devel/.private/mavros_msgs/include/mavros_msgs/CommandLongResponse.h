// Generated by gencpp from file mavros_msgs/CommandLongResponse.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_COMMANDLONGRESPONSE_H
#define MAVROS_MSGS_MESSAGE_COMMANDLONGRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mavros_msgs
{
template <class ContainerAllocator>
struct CommandLongResponse_
{
  typedef CommandLongResponse_<ContainerAllocator> Type;

  CommandLongResponse_()
    : success(false)
    , result(0)
    , result_param1(0)
    , result_param2(0)  {
    }
  CommandLongResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , result(0)
    , result_param1(0)
    , result_param2(0)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef uint8_t _result_type;
  _result_type result;

   typedef uint8_t _result_param1_type;
  _result_param1_type result_param1;

   typedef int32_t _result_param2_type;
  _result_param2_type result_param2;





  typedef boost::shared_ptr< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> const> ConstPtr;

}; // struct CommandLongResponse_

typedef ::mavros_msgs::CommandLongResponse_<std::allocator<void> > CommandLongResponse;

typedef boost::shared_ptr< ::mavros_msgs::CommandLongResponse > CommandLongResponsePtr;
typedef boost::shared_ptr< ::mavros_msgs::CommandLongResponse const> CommandLongResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::CommandLongResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::CommandLongResponse_<ContainerAllocator1> & lhs, const ::mavros_msgs::CommandLongResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.result == rhs.result &&
    lhs.result_param1 == rhs.result_param1 &&
    lhs.result_param2 == rhs.result_param2;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::CommandLongResponse_<ContainerAllocator1> & lhs, const ::mavros_msgs::CommandLongResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "05a1be4ea1d4023975a944262b6a822f";
  }

  static const char* value(const ::mavros_msgs::CommandLongResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x05a1be4ea1d40239ULL;
  static const uint64_t static_value2 = 0x75a944262b6a822fULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/CommandLongResponse";
  }

  static const char* value(const ::mavros_msgs::CommandLongResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"# raw result returned by COMMAND_ACK\n"
"uint8 result\n"
"uint8 result_param1\n"
"int32 result_param2\n"
"\n"
;
  }

  static const char* value(const ::mavros_msgs::CommandLongResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.result);
      stream.next(m.result_param1);
      stream.next(m.result_param2);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommandLongResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::CommandLongResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::CommandLongResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result);
    s << indent << "result_param1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.result_param1);
    s << indent << "result_param2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.result_param2);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_COMMANDLONGRESPONSE_H