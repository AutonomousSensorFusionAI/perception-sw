// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOXES__BUILDER_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOXES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msgs_bbox/msg/detail/b_boxes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msgs_bbox
{

namespace msg
{

namespace builder
{

class Init_BBoxes_bbox
{
public:
  Init_BBoxes_bbox()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::msgs_bbox::msg::BBoxes bbox(::msgs_bbox::msg::BBoxes::_bbox_type arg)
  {
    msg_.bbox = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs_bbox::msg::BBoxes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs_bbox::msg::BBoxes>()
{
  return msgs_bbox::msg::builder::Init_BBoxes_bbox();
}

}  // namespace msgs_bbox

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOXES__BUILDER_HPP_
