#ifndef MavLinkCom_UdpSocketImpl_hpp
#define MavLinkCom_UdpSocketImpl_hpp

#include "Utils.hpp"
#include "UdpSocket.hpp"
#ifdef _WIN32
// windows
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

typedef int socklen_t;
static bool socket_initialized_ = false;
#else

// posix
#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cerrno>
#include <netdb.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
typedef int SOCKET;
const int INVALID_SOCKET = -1;
const int ERROR_ACCESS_DENIED = EACCES;

inline int WSAGetLastError() {
    return errno;
}
const int SOCKET_ERROR = -1;
#define E_NOT_SUFFICIENT_BUFFER ENOMEM

#endif

using namespace mavlink_utils;
using namespace mavlinkcom;

namespace mavlinkcom_impl {

    class UdpSocketImpl
    {
    public:
        UdpSocketImpl();
        ~UdpSocketImpl();

        int connect(const std::string& address, uint16_t port);
        int bind(const std::string& address, uint16_t port);
        bool reuseaddress();
        void set_broadcast(void);

        int send(const void *pkt, size_t size);
        int sendto(const void *buf, size_t size, const std::string& address, uint16_t port);
        int recv(void *pkt, size_t size, uint32_t timeout_ms);

        // return the IP address and port of the last received packet
        void last_recv_address(std::string& ip_addr, uint16_t &port);

        void close();

    private:
        struct sockaddr_in in_addr {};

        SOCKET fd = -1;

        // return true if there is pending data for input
        bool pollin(uint32_t timeout_ms);

        // return true if there is room for output data
        bool pollout(uint32_t timeout_ms);

        void make_sockaddr(const std::string& address, uint16_t port, struct sockaddr_in &sockaddr);
    };

}

#endif
