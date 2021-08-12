// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from airsim_interfaces:msg/VelCmdGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__FUNCTIONS_H_
#define AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "airsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "airsim_interfaces/msg/detail/vel_cmd_group__struct.h"

/// Initialize msg/VelCmdGroup message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__msg__VelCmdGroup
 * )) before or use
 * airsim_interfaces__msg__VelCmdGroup__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__msg__VelCmdGroup__init(airsim_interfaces__msg__VelCmdGroup * msg);

/// Finalize msg/VelCmdGroup message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__VelCmdGroup__fini(airsim_interfaces__msg__VelCmdGroup * msg);

/// Create msg/VelCmdGroup message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__msg__VelCmdGroup__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__msg__VelCmdGroup *
airsim_interfaces__msg__VelCmdGroup__create();

/// Destroy msg/VelCmdGroup message.
/**
 * It calls
 * airsim_interfaces__msg__VelCmdGroup__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__VelCmdGroup__destroy(airsim_interfaces__msg__VelCmdGroup * msg);


/// Initialize array of msg/VelCmdGroup messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__msg__VelCmdGroup__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__msg__VelCmdGroup__Sequence__init(airsim_interfaces__msg__VelCmdGroup__Sequence * array, size_t size);

/// Finalize array of msg/VelCmdGroup messages.
/**
 * It calls
 * airsim_interfaces__msg__VelCmdGroup__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__VelCmdGroup__Sequence__fini(airsim_interfaces__msg__VelCmdGroup__Sequence * array);

/// Create array of msg/VelCmdGroup messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__msg__VelCmdGroup__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__msg__VelCmdGroup__Sequence *
airsim_interfaces__msg__VelCmdGroup__Sequence__create(size_t size);

/// Destroy array of msg/VelCmdGroup messages.
/**
 * It calls
 * airsim_interfaces__msg__VelCmdGroup__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__msg__VelCmdGroup__Sequence__destroy(airsim_interfaces__msg__VelCmdGroup__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__MSG__DETAIL__VEL_CMD_GROUP__FUNCTIONS_H_
