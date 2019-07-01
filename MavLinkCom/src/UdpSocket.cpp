#include "UdpSocket.hpp"
#include "impl/UdpSocketImpl.hpp"

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

UdpSocket::UdpSocket()
{
    pImpl.reset(new UdpSocketImpl());
}

int UdpSocket::connect(const std::string& addr, int port)
{
    return pImpl->connect(addr, port);
}

int UdpSocket::bind(const std::string& localaddr, int port)
{
    return pImpl->bind(localaddr, port);
}

void UdpSocket::close()
{
    pImpl->close();
}

int UdpSocket::send(const void *pkt, size_t size)
{
    return pImpl->send(pkt, size);
}

int UdpSocket::sendto(const void *buf, size_t size, const std::string& address, uint16_t port)
{
    return pImpl->sendto(buf, size, address, port);
}

int UdpSocket::recv(void *pkt, size_t size, uint32_t timeout_ms)
{
    return pImpl->recv(pkt, size, timeout_ms);
}

void UdpSocket::last_recv_address(std::string& ip_addr, uint16_t& port)
{
    pImpl->last_recv_address(ip_addr, port);
}

bool UdpSocket::reuseaddress()
{
    return pImpl->reuseaddress();
}

void UdpSocket::set_broadcast(void)
{
    pImpl->set_broadcast();
}

UdpSocket::~UdpSocket() {
    pImpl->close();
    pImpl = nullptr;
}
