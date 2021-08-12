// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/altimeter__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
airsim_interfaces__msg__Altimeter__init(airsim_interfaces__msg__Altimeter * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    airsim_interfaces__msg__Altimeter__fini(msg);
    return false;
  }
  // altitude
  // pressure
  // qnh
  return true;
}

void
airsim_interfaces__msg__Altimeter__fini(airsim_interfaces__msg__Altimeter * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // altitude
  // pressure
  // qnh
}

airsim_interfaces__msg__Altimeter *
airsim_interfaces__msg__Altimeter__create()
{
  airsim_interfaces__msg__Altimeter * msg = (airsim_interfaces__msg__Altimeter *)malloc(sizeof(airsim_interfaces__msg__Altimeter));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__Altimeter));
  bool success = airsim_interfaces__msg__Altimeter__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__Altimeter__destroy(airsim_interfaces__msg__Altimeter * msg)
{
  if (msg) {
    airsim_interfaces__msg__Altimeter__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__msg__Altimeter__Sequence__init(airsim_interfaces__msg__Altimeter__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__msg__Altimeter * data = NULL;
  if (size) {
    data = (airsim_interfaces__msg__Altimeter *)calloc(size, sizeof(airsim_interfaces__msg__Altimeter));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__Altimeter__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__Altimeter__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
airsim_interfaces__msg__Altimeter__Sequence__fini(airsim_interfaces__msg__Altimeter__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__msg__Altimeter__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

airsim_interfaces__msg__Altimeter__Sequence *
airsim_interfaces__msg__Altimeter__Sequence__create(size_t size)
{
  airsim_interfaces__msg__Altimeter__Sequence * array = (airsim_interfaces__msg__Altimeter__Sequence *)malloc(sizeof(airsim_interfaces__msg__Altimeter__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__Altimeter__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__Altimeter__Sequence__destroy(airsim_interfaces__msg__Altimeter__Sequence * array)
{
  if (array) {
    airsim_interfaces__msg__Altimeter__Sequence__fini(array);
  }
  free(array);
}
