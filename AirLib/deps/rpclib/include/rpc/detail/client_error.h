#pragma once

#ifndef RPC_CLIENT_ERROR_H
#define RPC_CLIENT_ERROR_H

#include <stdexcept>
#include <cstdint>

namespace rpc {
namespace detail {

//! \brief Describes an error that is the result of a connected client
//! doing something unexpected (e.g. calling a function that does not exist,
//! wrong number of arguments, etc.)
class client_error : public std::exception {
public:
    //! \brief Error codes used for signaling back to clients. These are used
    //! to produce google-able error messages (since the msgpack-rpc protocol
    //! does not define error handling in any more detail than sending an
    //! object).
    //! \note Care must be taken to keep these codes stable even between major
    //! versions.
    enum class code : uint16_t {
        no_such_function = 1,
        wrong_arity = 2,
        protocol_error = 4
    };

    client_error(code c, const std::string &msg);

    const char *what() const noexcept;

private:
    std::string what_;
};
}
}

#endif // RPC_CLIENT_ERROR_H
