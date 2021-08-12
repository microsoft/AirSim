// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from airsim_interfaces:srv/TakeoffGroup.idl
// generated code does not contain a copyright notice

#ifndef AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF_GROUP__FUNCTIONS_H_
#define AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF_GROUP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "airsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "airsim_interfaces/srv/detail/takeoff_group__struct.h"

/// Initialize srv/TakeoffGroup message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__srv__TakeoffGroup_Request
 * )) before or use
 * airsim_interfaces__srv__TakeoffGroup_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__TakeoffGroup_Request__init(airsim_interfaces__srv__TakeoffGroup_Request * msg);

/// Finalize srv/TakeoffGroup message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Request__fini(airsim_interfaces__srv__TakeoffGroup_Request * msg);

/// Create srv/TakeoffGroup message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__srv__TakeoffGroup_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__TakeoffGroup_Request *
airsim_interfaces__srv__TakeoffGroup_Request__create();

/// Destroy srv/TakeoffGroup message.
/**
 * It calls
 * airsim_interfaces__srv__TakeoffGroup_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Request__destroy(airsim_interfaces__srv__TakeoffGroup_Request * msg);


/// Initialize array of srv/TakeoffGroup messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__srv__TakeoffGroup_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__TakeoffGroup_Request__Sequence__init(airsim_interfaces__srv__TakeoffGroup_Request__Sequence * array, size_t size);

/// Finalize array of srv/TakeoffGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__TakeoffGroup_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Request__Sequence__fini(airsim_interfaces__srv__TakeoffGroup_Request__Sequence * array);

/// Create array of srv/TakeoffGroup messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__srv__TakeoffGroup_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__TakeoffGroup_Request__Sequence *
airsim_interfaces__srv__TakeoffGroup_Request__Sequence__create(size_t size);

/// Destroy array of srv/TakeoffGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__TakeoffGroup_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Request__Sequence__destroy(airsim_interfaces__srv__TakeoffGroup_Request__Sequence * array);

/// Initialize srv/TakeoffGroup message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * airsim_interfaces__srv__TakeoffGroup_Response
 * )) before or use
 * airsim_interfaces__srv__TakeoffGroup_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__TakeoffGroup_Response__init(airsim_interfaces__srv__TakeoffGroup_Response * msg);

/// Finalize srv/TakeoffGroup message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Response__fini(airsim_interfaces__srv__TakeoffGroup_Response * msg);

/// Create srv/TakeoffGroup message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * airsim_interfaces__srv__TakeoffGroup_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__TakeoffGroup_Response *
airsim_interfaces__srv__TakeoffGroup_Response__create();

/// Destroy srv/TakeoffGroup message.
/**
 * It calls
 * airsim_interfaces__srv__TakeoffGroup_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Response__destroy(airsim_interfaces__srv__TakeoffGroup_Response * msg);


/// Initialize array of srv/TakeoffGroup messages.
/**
 * It allocates the memory for the number of elements and calls
 * airsim_interfaces__srv__TakeoffGroup_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
bool
airsim_interfaces__srv__TakeoffGroup_Response__Sequence__init(airsim_interfaces__srv__TakeoffGroup_Response__Sequence * array, size_t size);

/// Finalize array of srv/TakeoffGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__TakeoffGroup_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Response__Sequence__fini(airsim_interfaces__srv__TakeoffGroup_Response__Sequence * array);

/// Create array of srv/TakeoffGroup messages.
/**
 * It allocates the memory for the array and calls
 * airsim_interfaces__srv__TakeoffGroup_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
airsim_interfaces__srv__TakeoffGroup_Response__Sequence *
airsim_interfaces__srv__TakeoffGroup_Response__Sequence__create(size_t size);

/// Destroy array of srv/TakeoffGroup messages.
/**
 * It calls
 * airsim_interfaces__srv__TakeoffGroup_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_airsim_interfaces
void
airsim_interfaces__srv__TakeoffGroup_Response__Sequence__destroy(airsim_interfaces__srv__TakeoffGroup_Response__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // AIRSIM_INTERFACES__SRV__DETAIL__TAKEOFF_GROUP__FUNCTIONS_H_
