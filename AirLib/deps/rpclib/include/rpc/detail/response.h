#pragma once

#ifndef RESPONSE_H_MVRZEKPX
#define RESPONSE_H_MVRZEKPX

#include "rpc/detail/log.h"
#include "rpc/msgpack.hpp"

namespace rpc {
namespace detail {

//! \brief Represents a response and creates a msgpack to be sent back
//! as per the msgpack-rpc spec.
class response {
public:
    //! \brief Creates a response that represents a normal return value.
    //! \param id The sequence id (as per protocol).
    //! \param result The return value to store in the response.
    //! \tparam T Any msgpack-able type.
    //! \note If there is both an error and result in the response,
    //! the result will be discarded while packing the data.
    template <typename T> static response make_result(uint32_t id, T &&result);

    //! \brief Creates a response that represents an error.
    //! \param id The sequence id (as per protocol).
    //! \param error The error value to store in the response.
    //! \tparam T Any msgpack-able type.
    template <typename T> static response make_error(uint32_t id, T &&error);


    //! \brief Constructs a response from RPCLIB_MSGPACK::object (useful when
    //! reading a response from a stream).
    response(RPCLIB_MSGPACK::object_handle o);

    //! \brief Gets the response data as a RPCLIB_MSGPACK::sbuffer.
    RPCLIB_MSGPACK::sbuffer get_data() const;

    //! \brief Moves the specified object_handle into the response
    //! as a result.
    //! \param r The result to capture.
    void capture_result(RPCLIB_MSGPACK::object_handle &r);

    //! \brief Moves the specified object_handle into the response as an error.
    //! \param e The error to capture.
    void capture_error(RPCLIB_MSGPACK::object_handle &e);

    //! \brief Returns the call id/index used to identify which call
    //! this response corresponds to.
    uint32_t get_id() const;

    //! \brief Returns the error object stored in the response. Can
    //! be empty.
    std::shared_ptr<RPCLIB_MSGPACK::object_handle> get_error() const;

    //! \brief Returns the result stored in the response. Can be empty.
    std::shared_ptr<RPCLIB_MSGPACK::object_handle> get_result() const;

    //! \brief Gets an empty response which means "no response" (not to be
    //! confused with void return, i.e. this means literally
    //! "don't write the response to the socket")
    static response empty();

    //! \brief If true, this response is empty (\see empty())
    bool is_empty() const;

    //! \brief The type of a response, according to the msgpack-rpc spec
    using response_type =
        std::tuple<uint32_t, uint32_t, RPCLIB_MSGPACK::object, RPCLIB_MSGPACK::object>;

private:
    //! \brief Default constructor for responses.
    response();

    uint32_t id_;
    // I really wish to avoid shared_ptr here but at this point asio does not
    // work with move-only handlers in post() and I need to capture responses
    // in lambdas.
    std::shared_ptr<RPCLIB_MSGPACK::object_handle> error_;
    std::shared_ptr<RPCLIB_MSGPACK::object_handle> result_;
    bool empty_;
    RPCLIB_CREATE_LOG_CHANNEL(response)
};

template <typename T>
inline response response::make_result(uint32_t id, T &&result) {
    auto z = std::make_unique<RPCLIB_MSGPACK::zone>();
    RPCLIB_MSGPACK::object o(std::forward<T>(result), *z);
    response inst;
    inst.id_ = id;
    inst.result_ = std::make_shared<RPCLIB_MSGPACK::object_handle>(o, std::move(z));
    return inst;
}

template <>
inline response
response::make_result(uint32_t id, std::unique_ptr<RPCLIB_MSGPACK::object_handle> &&r) {
    response inst;
    inst.id_ = id;
    inst.result_ = std::move(r);
    return inst;
}

template <typename T>
inline response response::make_error(uint32_t id, T &&error) {
    auto z = std::make_unique<RPCLIB_MSGPACK::zone>();
    RPCLIB_MSGPACK::object o(std::forward<T>(error), *z);
    response inst;
    inst.id_ = id;
    inst.error_ = std::make_shared<RPCLIB_MSGPACK::object_handle>(o, std::move(z));
    return inst;
}

} /* detail */

} /* rpc  */

#endif /* end of include guard: RESPONSE_H_MVRZEKPX */
