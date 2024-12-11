// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from msgs_bbox:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOX__BUILDER_HPP_
#define MSGS_BBOX__MSG__DETAIL__B_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "msgs_bbox/msg/detail/b_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace msgs_bbox
{

namespace msg
{

namespace builder
{

class Init_BBox_conf
{
public:
  explicit Init_BBox_conf(::msgs_bbox::msg::BBox & msg)
  : msg_(msg)
  {}
  ::msgs_bbox::msg::BBox conf(::msgs_bbox::msg::BBox::_conf_type arg)
  {
    msg_.conf = std::move(arg);
    return std::move(msg_);
  }

private:
  ::msgs_bbox::msg::BBox msg_;
};

class Init_BBox_y2
{
public:
  explicit Init_BBox_y2(::msgs_bbox::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_conf y2(::msgs_bbox::msg::BBox::_y2_type arg)
  {
    msg_.y2 = std::move(arg);
    return Init_BBox_conf(msg_);
  }

private:
  ::msgs_bbox::msg::BBox msg_;
};

class Init_BBox_x2
{
public:
  explicit Init_BBox_x2(::msgs_bbox::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_y2 x2(::msgs_bbox::msg::BBox::_x2_type arg)
  {
    msg_.x2 = std::move(arg);
    return Init_BBox_y2(msg_);
  }

private:
  ::msgs_bbox::msg::BBox msg_;
};

class Init_BBox_y1
{
public:
  explicit Init_BBox_y1(::msgs_bbox::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_x2 y1(::msgs_bbox::msg::BBox::_y1_type arg)
  {
    msg_.y1 = std::move(arg);
    return Init_BBox_x2(msg_);
  }

private:
  ::msgs_bbox::msg::BBox msg_;
};

class Init_BBox_x1
{
public:
  explicit Init_BBox_x1(::msgs_bbox::msg::BBox & msg)
  : msg_(msg)
  {}
  Init_BBox_y1 x1(::msgs_bbox::msg::BBox::_x1_type arg)
  {
    msg_.x1 = std::move(arg);
    return Init_BBox_y1(msg_);
  }

private:
  ::msgs_bbox::msg::BBox msg_;
};

class Init_BBox_cls
{
public:
  Init_BBox_cls()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BBox_x1 cls(::msgs_bbox::msg::BBox::_cls_type arg)
  {
    msg_.cls = std::move(arg);
    return Init_BBox_x1(msg_);
  }

private:
  ::msgs_bbox::msg::BBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::msgs_bbox::msg::BBox>()
{
  return msgs_bbox::msg::builder::Init_BBox_cls();
}

}  // namespace msgs_bbox

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOX__BUILDER_HPP_
