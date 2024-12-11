// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOXES__STRUCT_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOXES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'bbox'
#include "msgs_bbox/msg/detail/b_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__msgs_bbox__msg__BBoxes __attribute__((deprecated))
#else
# define DEPRECATED__msgs_bbox__msg__BBoxes __declspec(deprecated)
#endif

namespace msgs_bbox
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BBoxes_
{
  using Type = BBoxes_<ContainerAllocator>;

  explicit BBoxes_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BBoxes_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _bbox_type =
    std::vector<msgs_bbox::msg::BBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<msgs_bbox::msg::BBox_<ContainerAllocator>>>;
  _bbox_type bbox;

  // setters for named parameter idiom
  Type & set__bbox(
    const std::vector<msgs_bbox::msg::BBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<msgs_bbox::msg::BBox_<ContainerAllocator>>> & _arg)
  {
    this->bbox = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    msgs_bbox::msg::BBoxes_<ContainerAllocator> *;
  using ConstRawPtr =
    const msgs_bbox::msg::BBoxes_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      msgs_bbox::msg::BBoxes_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      msgs_bbox::msg::BBoxes_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__msgs_bbox__msg__BBoxes
    std::shared_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__msgs_bbox__msg__BBoxes
    std::shared_ptr<msgs_bbox::msg::BBoxes_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BBoxes_ & other) const
  {
    if (this->bbox != other.bbox) {
      return false;
    }
    return true;
  }
  bool operator!=(const BBoxes_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BBoxes_

// alias to use template instance with default allocator
using BBoxes =
  msgs_bbox::msg::BBoxes_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace msgs_bbox

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOXES__STRUCT_HPP_
