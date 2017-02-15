namespace rpc {

template <typename... Args>
RPCLIB_MSGPACK::object_handle client::call(std::string const &func_name,
                                    Args... args) {
    RPCLIB_CREATE_LOG_CHANNEL(client)
    auto future = async_call(func_name, std::forward<Args>(args)...);
    future.wait();
    return future.get();
}

template <typename... Args>
std::future<RPCLIB_MSGPACK::object_handle>
client::async_call(std::string const &func_name, Args... args) {
    RPCLIB_CREATE_LOG_CHANNEL(client)
    wait_conn();
    using RPCLIB_MSGPACK::object;
    LOG_DEBUG("Calling {}", func_name);

    auto args_obj = std::make_tuple(args...);
    const int idx = get_next_call_idx();
    auto call_obj =
        std::make_tuple(static_cast<uint8_t>(client::request_type::call), idx,
                        func_name, args_obj);

    auto buffer = std::make_shared<RPCLIB_MSGPACK::sbuffer>();
    RPCLIB_MSGPACK::pack(*buffer, call_obj);

    // TODO: Change to move semantics when asio starts supporting move-only
    // handlers in post(). [sztomi, 2016-02-14]
    auto p = std::make_shared<std::promise<RPCLIB_MSGPACK::object_handle>>();
    auto ft = p->get_future();

    post(buffer, idx, func_name, p);

    return ft;
}

//! \brief Sends a notification with the given name and arguments (if any).
//! \param func_name The name of the notification to call.
//! \param args The arguments to pass to the function.
//! \note This function returns when the notification is written to the
//! socket.
//! \tparam Args THe types of the arguments.
template <typename... Args>
void client::send(std::string const &func_name, Args... args) {
    RPCLIB_CREATE_LOG_CHANNEL(client)
    LOG_DEBUG("Sending notification {}", func_name);

    auto args_obj = std::make_tuple(args...);
    auto call_obj = std::make_tuple(
        static_cast<uint8_t>(client::request_type::notification), func_name,
        args_obj);

    auto buffer = new RPCLIB_MSGPACK::sbuffer;
    RPCLIB_MSGPACK::pack(*buffer, call_obj);

    post(buffer);
}
}
