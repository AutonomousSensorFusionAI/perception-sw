// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs_bbox:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOX__STRUCT_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__msgs_bbox__msg__BBox __attribute__((deprecated))
#else
# define DEPRECATED__msgs_bbox__msg__BBox __declspec(deprecated)
#endif

namespace msgs_bbox
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BBox_
{
  using Type = BBox_<ContainerAllocator>;

  explicit BBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cls = 0l;
      this->x1 = 0l;
      this->y1 = 0l;
      this->x2 = 0l;
      this->y2 = 0l;
      this->conf = 0.0f;
    }
  }

  explicit BBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cls = 0l;
      this->x1 = 0l;
      this->y1 = 0l;
      this->x2 = 0l;
      this->y2 = 0l;
      this->conf = 0.0f;
    }
  }

  // field types and members
  using _cls_type =
    int32_t;
  _cls_type cls;
  using _x1_type =
    int32_t;
  _x1_type x1;
  using _y1_type =
    int32_t;
  _y1_type y1;
  using _x2_type =
    int32_t;
  _x2_type x2;
  using _y2_type =
    int32_t;
  _y2_type y2;
  using _conf_type =
    float;
  _conf_type conf;

  // setters for named parameter idiom
  Type & set__cls(
    const int32_t & _arg)
  {
    this->cls = _arg;
    return *this;
  }
  Type & set__x1(
    const int32_t & _arg)
  {
    this->x1 = _arg;
    return *this;
  }
  Type & set__y1(
    const int32_t & _arg)
  {
    this->y1 = _arg;
    return *this;
  }
  Type & set__x2(
    const int32_t & _arg)
  {
    this->x2 = _arg;
    return *this;
  }
  Type & set__y2(
    const int32_t & _arg)
  {
    this->y2 = _arg;
    return *this;
  }
  Type & set__conf(
    const float & _arg)
  {
    this->conf = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs_bbox::msg::BBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs_bbox::msg::BBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs_bbox::msg::BBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs_bbox::msg::BBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs_bbox::msg::BBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs_bbox::msg::BBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs_bbox::msg::BBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs_bbox::msg::BBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs_bbox::msg::BBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs_bbox::msg::BBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs_bbox__msg__BBox
    std::shared_ptr<msgs_bbox::msg::BBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs_bbox__msg__BBox
    std::shared_ptr<msgs_bbox::msg::BBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BBox_ & other) const
  {
    if (this->cls != other.cls) {
      return false;
    }
    if (this->x1 != other.x1) {
      return false;
    }
    if (this->y1 != other.y1) {
      return false;
    }
    if (this->x2 != other.x2) {
      return false;
    }
    if (this->y2 != other.y2) {
      return false;
    }
    if (this->conf != other.conf) {
      return false;
    }
    return true;
  }
  bool operator!=(const BBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BBox_

// alias to use template instance with default allocator
using BBox =
  msgs_bbox::msg::BBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msgs_bbox

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOX__STRUCT_HPP_
