// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from airsim_interfaces:msg/GPSYaw.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__FUNCTIONS_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "airsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "airsim_interfaces/msg/detail/gps_yaw__struct.h"

/// Initialize msg/GPSYaw message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__msg__GPSYaw
 * )) before or use
 * airsim_interfaces__msg__GPSYaw__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__msg__GPSYaw__init(airsim_interfaces__msg__GPSYaw * msg);

/// Finalize msg/GPSYaw message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GPSYaw__fini(airsim_interfaces__msg__GPSYaw * msg);

/// Create msg/GPSYaw message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__msg__GPSYaw__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__msg__GPSYaw *
airsim_interfaces__msg__GPSYaw__create();

/// Destroy msg/GPSYaw message.
/**
 * It calls
 * airsim_interfaces__msg__GPSYaw__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GPSYaw__destroy(airsim_interfaces__msg__GPSYaw * msg);


/// Initialize array of msg/GPSYaw messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__msg__GPSYaw__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__msg__GPSYaw__Sequence__init(airsim_interfaces__msg__GPSYaw__Sequence * array, size_t size);

/// Finalize array of msg/GPSYaw messages.
/**
 * It calls
 * airsim_interfaces__msg__GPSYaw__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GPSYaw__Sequence__fini(airsim_interfaces__msg__GPSYaw__Sequence * array);

/// Create array of msg/GPSYaw messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__msg__GPSYaw__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__msg__GPSYaw__Sequence *
airsim_interfaces__msg__GPSYaw__Sequence__create(size_t size);

/// Destroy array of msg/GPSYaw messages.
/**
 * It calls
 * airsim_interfaces__msg__GPSYaw__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GPSYaw__Sequence__destroy(airsim_interfaces__msg__GPSYaw__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GPS_YAW__FUNCTIONS_H_
