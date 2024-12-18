// Generated by gencpp from file optoforce_ros/OptoForceFeedback.msg
// DO NOT EDIT!


#ifndef OPTOFORCE_ROS_MESSAGE_OPTOFORCEFEEDBACK_H
#define OPTOFORCE_ROS_MESSAGE_OPTOFORCEFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/WrenchStamped.h>

namespace optoforce_ros
{
template <class ContainerAllocator>
struct OptoForceFeedback_
{
  typedef OptoForceFeedback_<ContainerAllocator> Type;

  OptoForceFeedback_()
    : wrench_lst()  {
    }
  OptoForceFeedback_(const ContainerAllocator& _alloc)
    : wrench_lst(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::WrenchStamped_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::geometry_msgs::WrenchStamped_<ContainerAllocator> >> _wrench_lst_type;
  _wrench_lst_type wrench_lst;





  typedef boost::shared_ptr< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct OptoForceFeedback_

typedef ::optoforce_ros::OptoForceFeedback_<std::allocator<void> > OptoForceFeedback;

typedef boost::shared_ptr< ::optoforce_ros::OptoForceFeedback > OptoForceFeedbackPtr;
typedef boost::shared_ptr< ::optoforce_ros::OptoForceFeedback const> OptoForceFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator1> & lhs, const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.wrench_lst == rhs.wrench_lst;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator1> & lhs, const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace optoforce_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "33beeb9f57ea2f43c06c3f58af305729";
  }

  static const char* value(const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x33beeb9f57ea2f43ULL;
  static const uint64_t static_value2 = 0xc06c3f58af305729ULL;
};

template<class ContainerAllocator>
struct DataType< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optoforce_ros/OptoForceFeedback";
  }

  static const char* value(const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback\n"
"geometry_msgs/WrenchStamped[] wrench_lst\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/WrenchStamped\n"
"# A wrench with reference coordinate frame and timestamp\n"
"Header header\n"
"Wrench wrench\n"
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
"MSG: geometry_msgs/Wrench\n"
"# This represents force in free space, separated into\n"
"# its linear and angular parts.\n"
"Vector3  force\n"
"Vector3  torque\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.wrench_lst);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OptoForceFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optoforce_ros::OptoForceFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optoforce_ros::OptoForceFeedback_<ContainerAllocator>& v)
  {
    s << indent << "wrench_lst[]" << std::endl;
    for (size_t i = 0; i < v.wrench_lst.size(); ++i)
    {
      s << indent << "  wrench_lst[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::WrenchStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.wrench_lst[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTOFORCE_ROS_MESSAGE_OPTOFORCEFEEDBACK_H
