// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef MavLinkCom_MavLinkComImpl_hpp
#define MavLinkCom_MavLinkComImpl_hpp

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include "MavLinkNode.hpp"
#include "MavLinkNodeImpl.hpp"
#include "MavLinkFtpClient.hpp"
#include <stdio.h>
#include <string>

namespace mavlinkcom_impl {

	class MavLinkFtpClientImpl : public MavLinkNodeImpl
	{
	public:
		MavLinkFtpClientImpl(int local_system_id, int local_component_id);
		~MavLinkFtpClientImpl();

		bool isSupported();
		void list(MavLinkFtpProgress& progress, const std::string& remotePath, std::vector<MavLinkFileInfo>& files);
		void get(MavLinkFtpProgress& progress, const std::string& remotePath, const std::string& localPath);
		void put(MavLinkFtpProgress& progress, const std::string& remotePath, const std::string& localPath);
		void remove(MavLinkFtpProgress& progress, const std::string& remotePath);
		void cancel();
	private:
		void nextStep();
		void listDirectory();
		void removeFile();
		void readFile();
		void writeFile();
		void handleListResponse();
		void handleReadResponse();
		void handleWriteResponse();
		void handleRemoveResponse();
		void reset();
		void handleResponse(const MavLinkMessage& msg);
		bool createLocalFile();
		bool openSourceFile();
		void subscribe();
		void close();
		void recordMessageSent();
		void recordMessageReceived();
		void runStateMachine();
		void retry();

		// the following state is needed to implement a restartable state machine for each command
		// with a watchdog
		enum FtpCommandEnum {
			FtpCommandNone, FtpCommandList, FtpCommandGet, FtpCommandPut, FtpCommandRemove
		};
		FtpCommandEnum command_ = FtpCommandNone;
		std::string local_file_;
		std::string remote_file_;
		FILE* file_ptr_ = nullptr;
		int bytes_read_ = 0;
		bool remote_file_open_ = false;
		int bytes_written_ = 0;
		uint64_t file_size_ = 0;
		uint32_t file_index_ = 0;
		int sequence_ = 0;
		int subscription_ = 0;
		bool waiting_ = false;
		bool success_ = false;
		int errorCode_ = 0;
		std::chrono::milliseconds start_time_;
		std::chrono::milliseconds total_time_;
		int messages_ = 0;
		int retries_ = 0;
		MavLinkFileTransferProtocol last_message_;
		bool watch_dog_running_ = false;
		std::mutex mutex_;
		std::vector<mavlinkcom::MavLinkFileInfo>* files_ = nullptr;
		MavLinkFtpProgress* progress_ = nullptr;
	};
}

#endif
