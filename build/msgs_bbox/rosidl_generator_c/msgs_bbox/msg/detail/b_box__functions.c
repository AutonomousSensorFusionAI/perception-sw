// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from msgs_bbox:msg/BBox.idl
// generated code does not contain a copyright notice
#include "msgs_bbox/msg/detail/b_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
msgs_bbox__msg__BBox__init(msgs_bbox__msg__BBox * msg)
{
  if (!msg) {
    return false;
  }
  // cls
  // x1
  // y1
  // x2
  // y2
  // conf
  return true;
}

void
msgs_bbox__msg__BBox__fini(msgs_bbox__msg__BBox * msg)
{
  if (!msg) {
    return;
  }
  // cls
  // x1
  // y1
  // x2
  // y2
  // conf
}

bool
msgs_bbox__msg__BBox__are_equal(const msgs_bbox__msg__BBox * lhs, const msgs_bbox__msg__BBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cls
  if (lhs->cls != rhs->cls) {
    return false;
  }
  // x1
  if (lhs->x1 != rhs->x1) {
    return false;
  }
  // y1
  if (lhs->y1 != rhs->y1) {
    return false;
  }
  // x2
  if (lhs->x2 != rhs->x2) {
    return false;
  }
  // y2
  if (lhs->y2 != rhs->y2) {
    return false;
  }
  // conf
  if (lhs->conf != rhs->conf) {
    return false;
  }
  return true;
}

bool
msgs_bbox__msg__BBox__copy(
  const msgs_bbox__msg__BBox * input,
  msgs_bbox__msg__BBox * output)
{
  if (!input || !output) {
    return false;
  }
  // cls
  output->cls = input->cls;
  // x1
  output->x1 = input->x1;
  // y1
  output->y1 = input->y1;
  // x2
  output->x2 = input->x2;
  // y2
  output->y2 = input->y2;
  // conf
  output->conf = input->conf;
  return true;
}

msgs_bbox__msg__BBox *
msgs_bbox__msg__BBox__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs_bbox__msg__BBox * msg = (msgs_bbox__msg__BBox *)allocator.allocate(sizeof(msgs_bbox__msg__BBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(msgs_bbox__msg__BBox));
  bool success = msgs_bbox__msg__BBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
msgs_bbox__msg__BBox__destroy(msgs_bbox__msg__BBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    msgs_bbox__msg__BBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
msgs_bbox__msg__BBox__Sequence__init(msgs_bbox__msg__BBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs_bbox__msg__BBox * data = NULL;

  if (size) {
    data = (msgs_bbox__msg__BBox *)allocator.zero_allocate(size, sizeof(msgs_bbox__msg__BBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = msgs_bbox__msg__BBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        msgs_bbox__msg__BBox__fini(&data[i - 1]);
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
msgs_bbox__msg__BBox__Sequence__fini(msgs_bbox__msg__BBox__Sequence * array)
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
      msgs_bbox__msg__BBox__fini(&array->data[i]);
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

msgs_bbox__msg__BBox__Sequence *
msgs_bbox__msg__BBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  msgs_bbox__msg__BBox__Sequence * array = (msgs_bbox__msg__BBox__Sequence *)allocator.allocate(sizeof(msgs_bbox__msg__BBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = msgs_bbox__msg__BBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
msgs_bbox__msg__BBox__Sequence__destroy(msgs_bbox__msg__BBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    msgs_bbox__msg__BBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
msgs_bbox__msg__BBox__Sequence__are_equal(const msgs_bbox__msg__BBox__Sequence * lhs, const msgs_bbox__msg__BBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!msgs_bbox__msg__BBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
msgs_bbox__msg__BBox__Sequence__copy(
  const msgs_bbox__msg__BBox__Sequence * input,
  msgs_bbox__msg__BBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(msgs_bbox__msg__BBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    msgs_bbox__msg__BBox * data =
      (msgs_bbox__msg__BBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!msgs_bbox__msg__BBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          msgs_bbox__msg__BBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!msgs_bbox__msg__BBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
