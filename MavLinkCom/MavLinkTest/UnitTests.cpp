// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "UnitTests.h"
#include <thread>
#include <chrono>
#include "Utils.hpp"
#include "MavLinkVehicle.hpp"
#include "MavLinkMessages.hpp"
#include "MavLinkConnection.hpp"
#include "MavLinkVideoStream.hpp"
#include "MavLinkTcpServer.hpp"
#include "MavLinkFtpClient.hpp"
#include <thread>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include "MavLinkSemaphore.hpp"
#include <iostream>

using namespace common_utils;
using namespace mavlinkcom;

void UnitTests::RunAll(std::string comPort, int boardRate)
{
	com_port_ = comPort;
	baud_rate_ = boardRate;
	if (comPort == "") {
		throw std::runtime_error("unit tests need a serial connection to Pixhawk, please specify -serial argument");
	}

	RunTest("UdpPingTest", [=] { UdpPingTest(); });
	RunTest("TcpPingTest", [=] { TcpPingTest(); });
	RunTest("SendImageTest", [=] { SendImageTest(); });
	RunTest("SerialPx4Test", [=] { SerialPx4Test(); });
	RunTest("FtpTest", [=] { FtpTest(); });
}

void UnitTests::RunTest(const std::string& name, TestHandler handler)
{
	printf("%s: start\n", name.c_str());
	try {
		handler();
		printf("------------- %s test passed --------------------\n", name.c_str());
	}
	catch (const std::exception& e) {
		printf("------------- %s test failed: %s --------------- \n", name.c_str(), e.what());
	}
}

void UnitTests::UdpPingTest() {

	auto localConnection = MavLinkConnection::connectLocalUdp("jMavSim", "127.0.0.1", 14588);

	MavLinkSemaphore  received;
	auto id = localConnection->subscribe([&](std::shared_ptr<MavLinkConnection> connection, const MavLinkMessage& msg) {
		printf("    Received message %d\n", msg.msgid);
		received.post();
	});

	auto remoteConnection = MavLinkConnection::connectRemoteUdp("jMavSim", "127.0.0.1", "127.0.0.1", 14588);

	// send a heartbeat
	MavLinkHeartbeat hb;
	hb.autopilot = 0;
	hb.base_mode = 0;
	hb.custom_mode = 0;
	hb.mavlink_version = 3;
	hb.system_status = 1;
	hb.type = 1;

	auto node = std::make_shared<MavLinkNode>(166, 1);
	node->connect(remoteConnection);
	node->sendMessage(hb);

	if (!received.timed_wait(2000)) {
		throw std::runtime_error("heartbeat not received after 2 seconds");
	}
	localConnection->unsubscribe(id);
	localConnection->close();
	remoteConnection->close();
	node->close();
}

void UnitTests::TcpPingTest() {

	const int testPort = 45166;

	std::shared_ptr<MavLinkTcpServer> server = std::make_shared<MavLinkTcpServer>("127.0.0.1", testPort);
	std::shared_ptr<MavLinkNode> serverNode;

	server->acceptTcp("test", [&](std::shared_ptr<MavLinkConnection> con) {

		serverNode = std::make_shared<MavLinkNode>(1, 1);
		serverNode->connect(con);

		// send a heartbeat to the client
		MavLinkHeartbeat hb;
		hb.autopilot = 0;
		hb.base_mode = 0;
		hb.custom_mode = 0;
		hb.mavlink_version = 3;
		hb.system_status = 1;
		hb.type = 1;
		serverNode->sendMessage(hb);
	});

	MavLinkSemaphore  received;
	auto client = MavLinkConnection::connectTcp("local", "127.0.0.1", "127.0.0.1", testPort);
	client->subscribe([&](std::shared_ptr<MavLinkConnection> connection, const MavLinkMessage& msg) {
		printf("Received msg %d\n", msg.msgid);
		received.post();
	});

	if (!received.timed_wait(2000)) {
		throw std::runtime_error("heartbeat not received from server");
	}

	client->close();
}

void UnitTests::SerialPx4Test()
{
	auto connection = MavLinkConnection::connectSerial("px4", com_port_, baud_rate_);

	int count = 0;
	MavLinkSemaphore  received;

	auto id = connection->subscribe([&](std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg) {
		//printf("    Received message %d\n", static_cast<int>(msg.msgid));
		count++;
		if (msg.msgid == 0) {
			received.post();
		}
	});

	if (!received.timed_wait(5000)) {
		throw std::runtime_error("MavLink heartbeat is not being received over serial, please make sure PX4 is plugged in and the unit test is using the right COM port.");
	}

	printf("Received %d mavlink messages over serial port\n", count);
	connection->unsubscribe(id);
	connection->close();
}

class ImageServer {
public:
	ImageServer(std::shared_ptr<MavLinkConnection> con)
	{
		stream = std::make_shared<MavLinkVideoServer>(1, 1);
		stream->connect(con);
		con->subscribe([&](std::shared_ptr<MavLinkConnection> connection, const MavLinkMessage& msg) {
			
			MavLinkVideoServer::MavLinkVideoRequest req;
			if (stream->hasVideoRequest(req))
			{
				printf("    server received request for video at %f frames every second\n", req.every_n_sec);
			
				int* image = new int[10000];
				int size = sizeof(int) * 10000;
				for (int i = 0; i < 10000; i++)
				{
					image[i] = i;
				}
				stream->sendFrame(reinterpret_cast<uint8_t*>(image), size, 100, 100, 0, 0);
				delete image;
			}
		});
	}

	std::shared_ptr<MavLinkVideoServer> stream;
};

void UnitTests::SendImageTest() {

	const int testPort = 42316;
	std::string testAddr = "127.0.0.1";

	std::shared_ptr<MavLinkTcpServer> server = std::make_shared<MavLinkTcpServer>(testAddr, testPort);

	// this is the server code, it will accept 1 connection from a client on port 14588
	// for this unit test we are expecting a request to send an image.
	server->acceptTcp("test", [&](std::shared_ptr<MavLinkConnection> con) {
		this->server_ = new ImageServer(con);
	});

	// add a drone connection so the mavLinkCom can use it to send requests to the above server.
	auto drone = MavLinkConnection::connectTcp("drone", testAddr, testAddr, testPort);

	MavLinkVideoClient client{ 150, 1 };
	client.connect(drone);
	client.requestVideo(1, 1, false);

	MavLinkVideoClient::MavLinkVideoFrame image;
	int retries = 100;
	while (!client.readNextFrame(image) && retries-- > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	if (retries <= 0) {
		// hmmm
		throw std::runtime_error("no image received after timeout");
	}
	else {
		std::vector<unsigned char> raw = image.data;
		int* img = reinterpret_cast<int*>(raw.data());

		for (int i = 0, n = static_cast<int>(raw.size() / 4); i < n; i++)
		{
			if (img[i] != i) {
				throw std::runtime_error("corrupt image data received");
			}
		}
	}
	printf("    Received image %d bytes, width %d and height %d ok\n", static_cast<int>(image.data.size()), image.width, image.height);

	return;
}

void UnitTests::FtpTest() {

	auto connection = MavLinkConnection::connectSerial("px4", com_port_, baud_rate_);

	MavLinkFtpClient ftp{ 166,1 };
	ftp.connect(connection);

	MavLinkFtpProgress progress;
	std::vector<MavLinkFileInfo> files;
	ftp.list(progress, "/fs/microsd", files);
	if (progress.error != 0) {
		throw std::runtime_error(Utils::stringf("unexpected error %d: '%s' from ftp list '/fs/microsd' command - does your pixhawk have an sd card?",
			progress.error, progress.message.c_str()));
	}
	else 
	{
		printf("Found %d files in '/fs/microsd' folder\n", static_cast<int>(files.size()));
	}

	auto tempPath = boost::filesystem::temp_directory_path();
	tempPath.append("ftptest.txt", boost::filesystem::path::codecvt());
	boost::filesystem::ofstream stream(tempPath);

	const char* TestPattern = "This is line %d\n";

	for (int i = 0; i < 100; i++) {
		std::string line = Utils::stringf(TestPattern, i);
		stream << line;
	}

	stream.close();

	std::string remotePath = "/fs/microsd/ftptest.txt";
	std::string localPath = tempPath.generic_string();
#if defined(_WIN32)
	// I wish there was a cleaner way to do this, but I can't use tempPath.native() because on windows that is a wstring and on our linux build it is a string.
	boost::replace_all(localPath, "/", "\\");
#endif

	ftp.put(progress, remotePath, localPath);

	if (progress.error != 0) {
		throw std::runtime_error(Utils::stringf("unexpected error %d: '%s' from ftp put command",
			progress.error, progress.message.c_str()));
	}
	else
	{
		printf("put succeeded\n");
	}

	boost::filesystem::remove(tempPath);

	ftp.get(progress, remotePath, localPath);

	if (progress.error != 0) {
		throw std::runtime_error(Utils::stringf("unexpected error %d: '%s' from ftp get command",
			progress.error, progress.message.c_str()));
	}

	// verify the file contents.
	boost::filesystem::ifstream istream(tempPath);

	int count = 0;
	std::string line;
	std::getline(istream, line);
	while (line.size() > 0) {
		line += '\n';
		std::string expected = Utils::stringf(TestPattern, count);
		if (line != expected)
		{
			throw std::runtime_error(Utils::stringf("ftp local file contains unexpected contents '%s' on line %d\n", line.c_str(), count));
		}
		count++;
		std::getline(istream, line);
	}

	printf("get succeeded\n");

	istream.close();
	boost::filesystem::remove(tempPath);

	ftp.remove(progress, remotePath);

	if (progress.error != 0) {
		throw std::runtime_error(Utils::stringf("unexpected error %d: '%s' from ftp remove command",
			progress.error, progress.message.c_str()));
	}
	else
	{
		printf("remove succeeded\n");
	}


}
