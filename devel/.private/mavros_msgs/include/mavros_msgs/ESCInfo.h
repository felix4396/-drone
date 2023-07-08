// Generated by gencpp from file mavros_msgs/ESCInfo.msg
// DO NOT EDIT!


#ifndef MAVROS_MSGS_MESSAGE_ESCINFO_H
#define MAVROS_MSGS_MESSAGE_ESCINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <mavros_msgs/ESCInfoItem.h>

namespace mavros_msgs
{
template <class ContainerAllocator>
struct ESCInfo_
{
  typedef ESCInfo_<ContainerAllocator> Type;

  ESCInfo_()
    : header()
    , counter(0)
    , count(0)
    , connection_type(0)
    , info(0)
    , esc_info()  {
    }
  ESCInfo_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , counter(0)
    , count(0)
    , connection_type(0)
    , info(0)
    , esc_info(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint16_t _counter_type;
  _counter_type counter;

   typedef uint8_t _count_type;
  _count_type count;

   typedef uint8_t _connection_type_type;
  _connection_type_type connection_type;

   typedef uint8_t _info_type;
  _info_type info;

   typedef std::vector< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >> _esc_info_type;
  _esc_info_type esc_info;





  typedef boost::shared_ptr< ::mavros_msgs::ESCInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mavros_msgs::ESCInfo_<ContainerAllocator> const> ConstPtr;

}; // struct ESCInfo_

typedef ::mavros_msgs::ESCInfo_<std::allocator<void> > ESCInfo;

typedef boost::shared_ptr< ::mavros_msgs::ESCInfo > ESCInfoPtr;
typedef boost::shared_ptr< ::mavros_msgs::ESCInfo const> ESCInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mavros_msgs::ESCInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mavros_msgs::ESCInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mavros_msgs::ESCInfo_<ContainerAllocator1> & lhs, const ::mavros_msgs::ESCInfo_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.counter == rhs.counter &&
    lhs.count == rhs.count &&
    lhs.connection_type == rhs.connection_type &&
    lhs.info == rhs.info &&
    lhs.esc_info == rhs.esc_info;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mavros_msgs::ESCInfo_<ContainerAllocator1> & lhs, const ::mavros_msgs::ESCInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mavros_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mavros_msgs::ESCInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mavros_msgs::ESCInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mavros_msgs::ESCInfo_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0dadbe17da7077cfe645928710920e5e";
  }

  static const char* value(const ::mavros_msgs::ESCInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0dadbe17da7077cfULL;
  static const uint64_t static_value2 = 0xe645928710920e5eULL;
};

template<class ContainerAllocator>
struct DataType< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros_msgs/ESCInfo";
  }

  static const char* value(const ::mavros_msgs::ESCInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ESCInfo.msg\n"
"#\n"
"#\n"
"# See mavlink message documentation here:\n"
"# https://mavlink.io/en/messages/common.html#ESC_INFO\n"
"\n"
"std_msgs/Header header\n"
"\n"
"uint16 counter\n"
"uint8 count\n"
"uint8 connection_type\n"
"uint8 info\n"
"mavros_msgs/ESCInfoItem[] esc_info\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: mavros_msgs/ESCInfoItem\n"
"# ESCInfoItem.msg\n"
"#\n"
"#\n"
"# See mavlink message documentation here:\n"
"# https://mavlink.io/en/messages/common.html#ESC_INFO\n"
"\n"
"std_msgs/Header header\n"
"\n"
"uint16 failure_flags\n"
"uint32 error_count\n"
"uint8 temperature\n"
"\n"
;
  }

  static const char* value(const ::mavros_msgs::ESCInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.counter);
      stream.next(m.count);
      stream.next(m.connection_type);
      stream.next(m.info);
      stream.next(m.esc_info);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ESCInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mavros_msgs::ESCInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mavros_msgs::ESCInfo_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "counter: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.counter);
    s << indent << "count: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.count);
    s << indent << "connection_type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.connection_type);
    s << indent << "info: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.info);
    s << indent << "esc_info[]" << std::endl;
    for (size_t i = 0; i < v.esc_info.size(); ++i)
    {
      s << indent << "  esc_info[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::mavros_msgs::ESCInfoItem_<ContainerAllocator> >::stream(s, indent + "    ", v.esc_info[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAVROS_MSGS_MESSAGE_ESCINFO_H
