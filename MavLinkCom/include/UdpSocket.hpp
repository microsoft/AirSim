#ifndef MavLinkCom_UdpSocket_hpp
#define MavLinkCom_UdpSocket_hpp

#include <string>
#include <memory>

namespace mavlinkcom_impl {
    class UdpSocketImpl;
}

namespace mavlinkcom {

    class UdpSocket;

    // This class represents a simple single-threaded Socket interface specific for Udp connections
    // Basic socket functions are exposed through this
    class UdpSocket
    {
    public:
        UdpSocket();

        // initiate a connection on a socket
        int connect(const std::string& addr, int port);

        // bind the socket to an address
        int bind(const std::string& localaddr, int port);

        void close();

        // Send message on a socket
        // Used when the socket is in a connected state (so that the intended recipient is known)
        int send(const void *pkt, size_t size);

        // Send message to the specified address, port
        int sendto(const void *buf, size_t size, const std::string& address, uint16_t port);

        // Receive message on socket
        int recv(void *pkt, size_t size, uint32_t timeout_ms);

        // return the IP address and port of the last received packet
        void last_recv_address(std::string& ip_addr, uint16_t &port);

        bool reuseaddress();
        void set_broadcast(void);

    public:
        //needed for piml pattern
        ~UdpSocket();

    private:
        std::unique_ptr<mavlinkcom_impl::UdpSocketImpl> pImpl;
        friend class mavlinkcom_impl::UdpSocketImpl;
    };
}

#endif
