
namespace rpc {

template <typename T> void this_handler_t::respond_error(T &&err_obj) {
    error_ = detail::pack(std::forward<T>(err_obj));
    throw detail::handler_error();
}

template <typename T> void this_handler_t::respond(T &&resp_obj) {
    resp_ = detail::pack(std::forward<T>(resp_obj));
    //throw detail::handler_spec_response();
}

} /* rpc */
