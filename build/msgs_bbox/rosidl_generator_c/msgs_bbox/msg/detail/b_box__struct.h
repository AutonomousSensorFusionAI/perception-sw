// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from msgs_bbox:msg/BBox.idl
// generated code does not contain a copyright notice

#ifndef MSGS_BBOX__MSG__DETAIL__B_BOX__STRUCT_H_
#define MSGS_BBOX__MSG__DETAIL__B_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BBox in the package msgs_bbox.
typedef struct msgs_bbox__msg__BBox
{
  int32_t cls;
  int32_t x1;
  int32_t y1;
  int32_t x2;
  int32_t y2;
  float conf;
} msgs_bbox__msg__BBox;

// Struct for a sequence of msgs_bbox__msg__BBox.
typedef struct msgs_bbox__msg__BBox__Sequence
{
  msgs_bbox__msg__BBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} msgs_bbox__msg__BBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MSGS_BBOX__MSG__DETAIL__B_BOX__STRUCT_H_
