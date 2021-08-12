// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/VelCmd.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/vel_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"

bool
airsim_interfaces__msg__VelCmd__init(airsim_interfaces__msg__VelCmd * msg)
{
  if (!msg) {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    airsim_interfaces__msg__VelCmd__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__msg__VelCmd__fini(airsim_interfaces__msg__VelCmd * msg)
{
  if (!msg) {
    return;
  }
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
}

airsim_interfaces__msg__VelCmd *
airsim_interfaces__msg__VelCmd__create()
{
  airsim_interfaces__msg__VelCmd * msg = (airsim_interfaces__msg__VelCmd *)malloc(sizeof(airsim_interfaces__msg__VelCmd));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__VelCmd));
  bool success = airsim_interfaces__msg__VelCmd__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__VelCmd__destroy(airsim_interfaces__msg__VelCmd * msg)
{
  if (msg) {
    airsim_interfaces__msg__VelCmd__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__msg__VelCmd__Sequence__init(airsim_interfaces__msg__VelCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__msg__VelCmd * data = NULL;
  if (size) {
    data = (airsim_interfaces__msg__VelCmd *)calloc(size, sizeof(airsim_interfaces__msg__VelCmd));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__VelCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__VelCmd__fini(&data[i - 1]);
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
airsim_interfaces__msg__VelCmd__Sequence__fini(airsim_interfaces__msg__VelCmd__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__msg__VelCmd__fini(&array->data[i]);
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

airsim_interfaces__msg__VelCmd__Sequence *
airsim_interfaces__msg__VelCmd__Sequence__create(size_t size)
{
  airsim_interfaces__msg__VelCmd__Sequence * array = (airsim_interfaces__msg__VelCmd__Sequence *)malloc(sizeof(airsim_interfaces__msg__VelCmd__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__VelCmd__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__VelCmd__Sequence__destroy(airsim_interfaces__msg__VelCmd__Sequence * array)
{
  if (array) {
    airsim_interfaces__msg__VelCmd__Sequence__fini(array);
  }
  free(array);
}
