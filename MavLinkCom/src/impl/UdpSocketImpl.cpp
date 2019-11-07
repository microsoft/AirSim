#ifndef MavLinkCom_UdpSocketImpl_cpp
#define MavLinkCom_UdpSocketImpl_cpp

#include "UdpSocketImpl.hpp"

using namespace mavlinkcom_impl;


UdpSocketImpl::UdpSocketImpl()   
{
    fd = socket(AF_INET, SOCK_DGRAM, 0);
}

void UdpSocketImpl::close()
{
    if (fd != -1) {
#ifdef _WIN32
        closesocket(fd);
#else
        ::close(fd);
#endif
        fd = -1;
    }
}

UdpSocketImpl::~UdpSocketImpl()
{
    close();
}

void UdpSocketImpl::make_sockaddr(const std::string& address, uint16_t port, struct sockaddr_in &sockaddr)
{
    memset(&sockaddr, 0, sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address.c_str());
}

/*
  connect the socket
 */
int UdpSocketImpl::connect(const std::string& address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    int rc = ::connect(fd, reinterpret_cast<struct sockaddr*>(&sockaddr), sizeof(sockaddr));
    if (rc != 0) {
        int hr = WSAGetLastError();
        throw std::runtime_error(Utils::stringf("UdpSocket connect failed with error: %d\n", hr));
        return hr;
    }
    return 0;
}

/*
  bind the socket
 */
int UdpSocketImpl::bind(const std::string& address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    int rc = ::bind(fd, reinterpret_cast<struct sockaddr*>(&sockaddr), sizeof(sockaddr));
    if (rc != 0)
    {
        int hr = WSAGetLastError();
        throw std::runtime_error(Utils::stringf("UdpSocket bind failed with error: %d\n", hr));
        return hr;
    }
    return 0;
}


/*
  set SO_REUSEADDR
 */
bool UdpSocketImpl::reuseaddress(void)
{
    int one = 1;
    return (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&one), sizeof(one)) != -1);
}

/*
  send some data
 */
int UdpSocketImpl::send(const void *buf, size_t size)
{
    int hr = ::send(fd, reinterpret_cast<const char*>(buf), static_cast<int>(size), 0);
    if (hr == SOCKET_ERROR)
    {
        hr = WSAGetLastError();
        throw std::runtime_error(Utils::stringf("Udp socket send failed with error: %d\n", hr));
    }
    return hr;
}

/*
  send some data
 */
int UdpSocketImpl::sendto(const void *buf, size_t size, const std::string& address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);
    int hr = ::sendto(fd, reinterpret_cast<const char*>(buf), static_cast<int>(size), 0, reinterpret_cast<struct sockaddr*>(&sockaddr), sizeof(sockaddr));
    if (hr == SOCKET_ERROR)
    {
        hr = WSAGetLastError();
        throw std::runtime_error(Utils::stringf("Udp socket send failed with error: %d\n", hr));
    }
    return hr;
}

/*
  receive some data
 */
int UdpSocketImpl::recv(void *buf, size_t size, uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        return -1;
    }
    socklen_t len = sizeof(in_addr);
    int rc = ::recvfrom(fd, reinterpret_cast<char*>(buf), static_cast<int>(size), 0, reinterpret_cast<sockaddr*>(&in_addr), &len);
    if (rc < 0)
    {
        rc = WSAGetLastError();
        Utils::log(Utils::stringf("Udp Socket recv failed with error: %d", rc));
    }
    return rc;
}

/*
  return the IP address and port of the last received packet
 */
void UdpSocketImpl::last_recv_address(std::string& ip_addr, uint16_t &port)
{
    ip_addr = inet_ntoa(in_addr.sin_addr);
    port = ntohs(in_addr.sin_port);
}

void UdpSocketImpl::set_broadcast(void)
{
    int one = 1;
    setsockopt(fd,SOL_SOCKET,SO_BROADCAST,reinterpret_cast<char*>(&one),sizeof(one));
}

/*
  return true if there is pending data for input
 */
bool UdpSocketImpl::pollin(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

#ifdef _WIN32
    if (select(0, &fds, nullptr, nullptr, &tv) != 1) {
#else
    if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
#endif
        return false;
    }
    return true;
}


/*
  return true if there is room for output data
 */
bool UdpSocketImpl::pollout(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

#ifdef _WIN32
    if (select(0, &fds, nullptr, nullptr, &tv) != 1) {
#else
    if (select(fd+1, &fds, nullptr, nullptr, &tv) != 1) {
#endif
        return false;
    }
    return true;
}


#endif // MavLinkCom_UdpSocketImpl_cpp
