// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#pragma once
#include <string>
#include <functional>

typedef std::function<void()> TestHandler;
class ImageServer;

class UnitTests {
  public:
    void RunAll(std::string comPort, int boardRate);
    void SerialPx4Test();
    void UdpPingTest();
    void TcpPingTest();
    void SendImageTest();
    void FtpTest();
  private:
    void RunTest(const std::string& name, TestHandler handler);
    ImageServer* server_;
    std::string com_port_;
    int baud_rate_;
};

