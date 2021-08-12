// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from airsim_interfaces:msg/GimbalAngleEulerCmd.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__FUNCTIONS_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "airsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "airsim_interfaces/msg/detail/gimbal_angle_euler_cmd__struct.h"

/// Initialize msg/GimbalAngleEulerCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__msg__GimbalAngleEulerCmd
 * )) before or use
 * airsim_interfaces__msg__GimbalAngleEulerCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__msg__GimbalAngleEulerCmd__init(airsim_interfaces__msg__GimbalAngleEulerCmd * msg);

/// Finalize msg/GimbalAngleEulerCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GimbalAngleEulerCmd__fini(airsim_interfaces__msg__GimbalAngleEulerCmd * msg);

/// Create msg/GimbalAngleEulerCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__msg__GimbalAngleEulerCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__msg__GimbalAngleEulerCmd *
airsim_interfaces__msg__GimbalAngleEulerCmd__create();

/// Destroy msg/GimbalAngleEulerCmd message.
/**
 * It calls
 * airsim_interfaces__msg__GimbalAngleEulerCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GimbalAngleEulerCmd__destroy(airsim_interfaces__msg__GimbalAngleEulerCmd * msg);


/// Initialize array of msg/GimbalAngleEulerCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__msg__GimbalAngleEulerCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence__init(airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence * array, size_t size);

/// Finalize array of msg/GimbalAngleEulerCmd messages.
/**
 * It calls
 * airsim_interfaces__msg__GimbalAngleEulerCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence__fini(airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence * array);

/// Create array of msg/GimbalAngleEulerCmd messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence *
airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence__create(size_t size);

/// Destroy array of msg/GimbalAngleEulerCmd messages.
/**
 * It calls
 * airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence__destroy(airsim_interfaces__msg__GimbalAngleEulerCmd__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__GIMBAL_ANGLE_EULER_CMD__FUNCTIONS_H_
