// Generated by Zakariae Mhiriz from file ros_zakariaem/EmgArray.msg
// DO NOT EDIT!


#ifndef ROS_ZAKARIAEM_MESSAGE_EMGARRAY_H
#define ROS_ZAKARIAEM_EMGARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ros_zakariaem
{
template <class ContainerAllocator>
struct EmgArray_
{
  typedef EmgArray_<ContainerAllocator> Type;

  EmgArray_()
    : header()
    , data()
    , moving(0)  {
      data.assign(0);
  }
  EmgArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , data()
    , moving(0)  {
  (void)_alloc;
      data.assign(0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array<int16_t, 8>  _data_type;
  _data_type data;

   typedef int16_t _moving_type;
  _moving_type moving;





  typedef boost::shared_ptr< ::ros_zakariaem::EmgArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_zakariaem::EmgArray_<ContainerAllocator> const> ConstPtr;

}; // struct EmgArray_

typedef ::ros_zakariaem::EmgArray_<std::allocator<void> > EmgArray;

typedef boost::shared_ptr< ::ros_zakariaem::EmgArray > EmgArrayPtr;
typedef boost::shared_ptr< ::ros_zakariaem::EmgArray const> EmgArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_zakariaem::EmgArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_zakariaem::EmgArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ros_zakariaem

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_zakariaem::EmgArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_zakariaem::EmgArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_zakariaem::EmgArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7aff6dea96e105d09f3a51f308d6c6d0";
  }

  static const char* value(const ::ros_zakariaem::EmgArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7aff6dea96e105d0ULL;
  static const uint64_t static_value2 = 0x9f3a51f308d6c6d0ULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_eehmd/EmgArray";
  }

  static const char* value(const ::ros_zakariaem::EmgArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# EmgArray message for the Thalmic Myo, which has 8 EMG sensors \n"
"# arranged around the arm\n"
"# There is a moving field that's unclear what it is looks like a bitmask\n"
"\n"
"Header header\n"
"int16[8] data\n"
"int16 moving\n"
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
;
  }

  static const char* value(const ::ros_zakariaem::EmgArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.data);
      stream.next(m.moving);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct EmgArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_zakariaem::EmgArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_zakariaem::EmgArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<int16_t>::stream(s, indent + "  ", v.data[i]);
    }
    s << indent << "moving: ";
    Printer<int16_t>::stream(s, indent + "  ", v.moving);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_zakariaem_MESSAGE_EMGARRAY_H
