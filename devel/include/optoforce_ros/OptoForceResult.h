// Generated by gencpp from file optoforce_ros/OptoForceResult.msg
// DO NOT EDIT!


#ifndef OPTOFORCE_ROS_MESSAGE_OPTOFORCERESULT_H
#define OPTOFORCE_ROS_MESSAGE_OPTOFORCERESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace optoforce_ros
{
template <class ContainerAllocator>
struct OptoForceResult_
{
  typedef OptoForceResult_<ContainerAllocator> Type;

  OptoForceResult_()
    : result(0)  {
    }
  OptoForceResult_(const ContainerAllocator& _alloc)
    : result(0)  {
  (void)_alloc;
    }



   typedef int32_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::optoforce_ros::OptoForceResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optoforce_ros::OptoForceResult_<ContainerAllocator> const> ConstPtr;

}; // struct OptoForceResult_

typedef ::optoforce_ros::OptoForceResult_<std::allocator<void> > OptoForceResult;

typedef boost::shared_ptr< ::optoforce_ros::OptoForceResult > OptoForceResultPtr;
typedef boost::shared_ptr< ::optoforce_ros::OptoForceResult const> OptoForceResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optoforce_ros::OptoForceResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::optoforce_ros::OptoForceResult_<ContainerAllocator1> & lhs, const ::optoforce_ros::OptoForceResult_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::optoforce_ros::OptoForceResult_<ContainerAllocator1> & lhs, const ::optoforce_ros::OptoForceResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace optoforce_ros

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optoforce_ros::OptoForceResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optoforce_ros::OptoForceResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optoforce_ros::OptoForceResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const ::optoforce_ros::OptoForceResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optoforce_ros/OptoForceResult";
  }

  static const char* value(const ::optoforce_ros::OptoForceResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#result definition\n"
"int32 result\n"
;
  }

  static const char* value(const ::optoforce_ros::OptoForceResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OptoForceResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optoforce_ros::OptoForceResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optoforce_ros::OptoForceResult_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<int32_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTOFORCE_ROS_MESSAGE_OPTOFORCERESULT_H