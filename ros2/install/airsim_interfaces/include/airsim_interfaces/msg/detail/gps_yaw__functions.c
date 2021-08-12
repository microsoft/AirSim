// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/msg/detail/gps_yaw__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
airsim_interfaces__msg__GPSYaw__init(airsim_interfaces__msg__GPSYaw * msg)
{
  if (!msg) {
    return false;
  }
  // latitude
  // longitude
  // altitude
  // yaw
  return true;
}

void
airsim_interfaces__msg__GPSYaw__fini(airsim_interfaces__msg__GPSYaw * msg)
{
  if (!msg) {
    return;
  }
  // latitude
  // longitude
  // altitude
  // yaw
}

airsim_interfaces__msg__GPSYaw *
airsim_interfaces__msg__GPSYaw__create()
{
  airsim_interfaces__msg__GPSYaw * msg = (airsim_interfaces__msg__GPSYaw *)malloc(sizeof(airsim_interfaces__msg__GPSYaw));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__msg__GPSYaw));
  bool success = airsim_interfaces__msg__GPSYaw__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__msg__GPSYaw__destroy(airsim_interfaces__msg__GPSYaw * msg)
{
  if (msg) {
    airsim_interfaces__msg__GPSYaw__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__msg__GPSYaw__Sequence__init(airsim_interfaces__msg__GPSYaw__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__msg__GPSYaw * data = NULL;
  if (size) {
    data = (airsim_interfaces__msg__GPSYaw *)calloc(size, sizeof(airsim_interfaces__msg__GPSYaw));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__msg__GPSYaw__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__msg__GPSYaw__fini(&data[i - 1]);
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
airsim_interfaces__msg__GPSYaw__Sequence__fini(airsim_interfaces__msg__GPSYaw__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__msg__GPSYaw__fini(&array->data[i]);
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

airsim_interfaces__msg__GPSYaw__Sequence *
airsim_interfaces__msg__GPSYaw__Sequence__create(size_t size)
{
  airsim_interfaces__msg__GPSYaw__Sequence * array = (airsim_interfaces__msg__GPSYaw__Sequence *)malloc(sizeof(airsim_interfaces__msg__GPSYaw__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__msg__GPSYaw__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__msg__GPSYaw__Sequence__destroy(airsim_interfaces__msg__GPSYaw__Sequence * array)
{
  if (array) {
    airsim_interfaces__msg__GPSYaw__Sequence__fini(array);
  }
  free(array);
}
