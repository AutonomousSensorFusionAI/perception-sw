// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msgs_bbox:msg/BBoxes.idl
// generated code does not contain a copyright notice
#include "msgs_bbox/msg/detail/b_boxes__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `bbox`
#include "msgs_bbox/msg/detail/b_box__functions.h"

bool
msgs_bbox__msg__BBoxes__init(msgs_bbox__msg__BBoxes * msg)
{
  if (!msg) {
    return false;
  }
  // bbox
  if (!msgs_bbox__msg__BBox__Sequence__init(&msg->bbox, 0)) {
    msgs_bbox__msg__BBoxes__fini(msg);
    return false;
  }
  return true;
}

void
msgs_bbox__msg__BBoxes__fini(msgs_bbox__msg__BBoxes * msg)
{
  if (!msg) {
    return;
  }
  // bbox
  msgs_bbox__msg__BBox__Sequence__fini(&msg->bbox);
}

bool
msgs_bbox__msg__BBoxes__are_equal(const msgs_bbox__msg__BBoxes * lhs, const msgs_bbox__msg__BBoxes * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bbox
  if (!msgs_bbox__msg__BBox__Sequence__are_equal(
      &(lhs->bbox), &(rhs->bbox)))
  {
    return false;
  }
  return true;
}

bool
msgs_bbox__msg__BBoxes__copy(
  const msgs_bbox__msg__BBoxes * input,
  msgs_bbox__msg__BBoxes * output)
{
  if (!input || !output) {
    return false;
  }
  // bbox
  if (!msgs_bbox__msg__BBox__Sequence__copy(
      &(input->bbox), &(output->bbox)))
  {
    return false;
  }
  return true;
}

msgs_bbox__msg__BBoxes *
msgs_bbox__msg__BBoxes__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs_bbox__msg__BBoxes * msg = (msgs_bbox__msg__BBoxes *)allocator.allocate(sizeof(msgs_bbox__msg__BBoxes), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msgs_bbox__msg__BBoxes));
  bool success = msgs_bbox__msg__BBoxes__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msgs_bbox__msg__BBoxes__destroy(msgs_bbox__msg__BBoxes * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msgs_bbox__msg__BBoxes__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msgs_bbox__msg__BBoxes__Sequence__init(msgs_bbox__msg__BBoxes__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs_bbox__msg__BBoxes * data = NULL;

  if (size) {
    data = (msgs_bbox__msg__BBoxes *)allocator.zero_allocate(size, sizeof(msgs_bbox__msg__BBoxes), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msgs_bbox__msg__BBoxes__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msgs_bbox__msg__BBoxes__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
msgs_bbox__msg__BBoxes__Sequence__fini(msgs_bbox__msg__BBoxes__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      msgs_bbox__msg__BBoxes__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

msgs_bbox__msg__BBoxes__Sequence *
msgs_bbox__msg__BBoxes__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs_bbox__msg__BBoxes__Sequence * array = (msgs_bbox__msg__BBoxes__Sequence *)allocator.allocate(sizeof(msgs_bbox__msg__BBoxes__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msgs_bbox__msg__BBoxes__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msgs_bbox__msg__BBoxes__Sequence__destroy(msgs_bbox__msg__BBoxes__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msgs_bbox__msg__BBoxes__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msgs_bbox__msg__BBoxes__Sequence__are_equal(const msgs_bbox__msg__BBoxes__Sequence * lhs, const msgs_bbox__msg__BBoxes__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msgs_bbox__msg__BBoxes__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msgs_bbox__msg__BBoxes__Sequence__copy(
  const msgs_bbox__msg__BBoxes__Sequence * input,
  msgs_bbox__msg__BBoxes__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msgs_bbox__msg__BBoxes);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msgs_bbox__msg__BBoxes * data =
      (msgs_bbox__msg__BBoxes *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msgs_bbox__msg__BBoxes__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msgs_bbox__msg__BBoxes__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msgs_bbox__msg__BBoxes__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
