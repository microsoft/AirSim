// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from airsim_interfaces:srv/SetGPSPosition.idl
// generated code does not contain a copyright notice
#include "airsim_interfaces/srv/detail/set_gps_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Include directives for member types
// Member `vehicle_name`
#include "rosidl_runtime_c/string_functions.h"

bool
airsim_interfaces__srv__SetGPSPosition_Request__init(airsim_interfaces__srv__SetGPSPosition_Request * msg)
{
  if (!msg) {
    return false;
  }
  // latitude
  // longitude
  // altitude
  // yaw
  // vehicle_name
  if (!rosidl_runtime_c__String__init(&msg->vehicle_name)) {
    airsim_interfaces__srv__SetGPSPosition_Request__fini(msg);
    return false;
  }
  return true;
}

void
airsim_interfaces__srv__SetGPSPosition_Request__fini(airsim_interfaces__srv__SetGPSPosition_Request * msg)
{
  if (!msg) {
    return;
  }
  // latitude
  // longitude
  // altitude
  // yaw
  // vehicle_name
  rosidl_runtime_c__String__fini(&msg->vehicle_name);
}

airsim_interfaces__srv__SetGPSPosition_Request *
airsim_interfaces__srv__SetGPSPosition_Request__create()
{
  airsim_interfaces__srv__SetGPSPosition_Request * msg = (airsim_interfaces__srv__SetGPSPosition_Request *)malloc(sizeof(airsim_interfaces__srv__SetGPSPosition_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__srv__SetGPSPosition_Request));
  bool success = airsim_interfaces__srv__SetGPSPosition_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__srv__SetGPSPosition_Request__destroy(airsim_interfaces__srv__SetGPSPosition_Request * msg)
{
  if (msg) {
    airsim_interfaces__srv__SetGPSPosition_Request__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__srv__SetGPSPosition_Request__Sequence__init(airsim_interfaces__srv__SetGPSPosition_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__srv__SetGPSPosition_Request * data = NULL;
  if (size) {
    data = (airsim_interfaces__srv__SetGPSPosition_Request *)calloc(size, sizeof(airsim_interfaces__srv__SetGPSPosition_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__srv__SetGPSPosition_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__srv__SetGPSPosition_Request__fini(&data[i - 1]);
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
airsim_interfaces__srv__SetGPSPosition_Request__Sequence__fini(airsim_interfaces__srv__SetGPSPosition_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__srv__SetGPSPosition_Request__fini(&array->data[i]);
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

airsim_interfaces__srv__SetGPSPosition_Request__Sequence *
airsim_interfaces__srv__SetGPSPosition_Request__Sequence__create(size_t size)
{
  airsim_interfaces__srv__SetGPSPosition_Request__Sequence * array = (airsim_interfaces__srv__SetGPSPosition_Request__Sequence *)malloc(sizeof(airsim_interfaces__srv__SetGPSPosition_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__srv__SetGPSPosition_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__srv__SetGPSPosition_Request__Sequence__destroy(airsim_interfaces__srv__SetGPSPosition_Request__Sequence * array)
{
  if (array) {
    airsim_interfaces__srv__SetGPSPosition_Request__Sequence__fini(array);
  }
  free(array);
}


bool
airsim_interfaces__srv__SetGPSPosition_Response__init(airsim_interfaces__srv__SetGPSPosition_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
airsim_interfaces__srv__SetGPSPosition_Response__fini(airsim_interfaces__srv__SetGPSPosition_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

airsim_interfaces__srv__SetGPSPosition_Response *
airsim_interfaces__srv__SetGPSPosition_Response__create()
{
  airsim_interfaces__srv__SetGPSPosition_Response * msg = (airsim_interfaces__srv__SetGPSPosition_Response *)malloc(sizeof(airsim_interfaces__srv__SetGPSPosition_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(airsim_interfaces__srv__SetGPSPosition_Response));
  bool success = airsim_interfaces__srv__SetGPSPosition_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
airsim_interfaces__srv__SetGPSPosition_Response__destroy(airsim_interfaces__srv__SetGPSPosition_Response * msg)
{
  if (msg) {
    airsim_interfaces__srv__SetGPSPosition_Response__fini(msg);
  }
  free(msg);
}


bool
airsim_interfaces__srv__SetGPSPosition_Response__Sequence__init(airsim_interfaces__srv__SetGPSPosition_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  airsim_interfaces__srv__SetGPSPosition_Response * data = NULL;
  if (size) {
    data = (airsim_interfaces__srv__SetGPSPosition_Response *)calloc(size, sizeof(airsim_interfaces__srv__SetGPSPosition_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = airsim_interfaces__srv__SetGPSPosition_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        airsim_interfaces__srv__SetGPSPosition_Response__fini(&data[i - 1]);
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
airsim_interfaces__srv__SetGPSPosition_Response__Sequence__fini(airsim_interfaces__srv__SetGPSPosition_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      airsim_interfaces__srv__SetGPSPosition_Response__fini(&array->data[i]);
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

airsim_interfaces__srv__SetGPSPosition_Response__Sequence *
airsim_interfaces__srv__SetGPSPosition_Response__Sequence__create(size_t size)
{
  airsim_interfaces__srv__SetGPSPosition_Response__Sequence * array = (airsim_interfaces__srv__SetGPSPosition_Response__Sequence *)malloc(sizeof(airsim_interfaces__srv__SetGPSPosition_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = airsim_interfaces__srv__SetGPSPosition_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
airsim_interfaces__srv__SetGPSPosition_Response__Sequence__destroy(airsim_interfaces__srv__SetGPSPosition_Response__Sequence * array)
{
  if (array) {
    airsim_interfaces__srv__SetGPSPosition_Response__Sequence__fini(array);
  }
  free(array);
}
