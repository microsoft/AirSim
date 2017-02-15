#pragma once

#ifndef HANDLER_H_BZ8DT5WS
#define HANDLER_H_BZ8DT5WS

#include <memory>

#include "rpc/msgpack.hpp"

#include "rpc/detail/util.h"

namespace rpc {

namespace detail {
class server_session;
class handler_error {};
class handler_spec_response {};
}

//! \brief Encapsulates information about the currently executing
//! handler. This is the interface through which bound functions
//! may return errors, arbitrary type responses or prohibit sending a response.
//! \note Setting each property of the handler is only relevant
//! for one call in one thread. If the same handler is executing concurrently
//! in a different thread, you can safely set different properties
//! and everything will "just work".
class this_handler_t {
public:
    //! \brief Sets an arbitrary object to be sent back as an error
    //! response to the client.
    //! \param err_obj The error object. This can be anything that
    //! is possible to encode with messagepack (even custom structures).
    //! \tparam T The type of the error object.
    template <typename T> void respond_error(T &&err_obj);

    //! \brief Sets an arbitrary object to be sent back as the response
    //! to the call.
    //! \param resp_obj The response object. This can be anything that
    //! is possible to encode with messagepack (even custom structures).
    //! \tparam T The type of the response object.
    //! \note The normal return value of the function (if any) will be
    //! ignored if a special response is set.
    //! \note You can use clear_special_response() to clear the special
    //! response and use the normal return value.
    template <typename T> void respond(T &&resp_obj);

    //! \brief Instructs the server to not send a response to the client
    //! (ignoring any errors and return values).
    //! \note It is unusual to not send a response to requests, and doing so
    //! might cause problems in the client (depending on its implementation).
    void disable_response();

    //! \brief Enables sending a response to the call. Sending the response
    //! is by default enabled. Enabling the response multiple times have
    //! no effect.
    void enable_response();

    //! \brief Sets all state of the object to default.
    void clear();

    friend class rpc::detail::server_session;

private:
    RPCLIB_MSGPACK::object_handle error_, resp_;
    bool resp_enabled_ = true;
};
}

#include "this_handler.inl"

namespace rpc {
//! \brief A thread-local object that can be used to control
//! the behavior of the server w.r.t. the handler. Accessing this object
//! from handlers that execute the same function concurrently is safe.
//! \note Accessing this object outside of handlers while a server is
//! running is potentially unsafe.
this_handler_t &this_handler();
}

#endif /* end of include guard: HANDLER_H_BZ8DT5WS */

