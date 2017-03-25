// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkFtpClientImpl.hpp"
#include <mutex>
#include "Utils.hpp"
#include "FileSystem.hpp"
#include <sys/stat.h>

using namespace mavlink_utils;
using namespace mavlinkcom;
using namespace mavlinkcom_impl;
using milliseconds = std::chrono::milliseconds;

#define MAXIMUM_ROUND_TRIP_TIME 200   // 200 milliseconds should be plenty of time for single round trip to remote node.
#define TIMEOUT_INTERVAL 10 // 10 * MAXIMUM_ROUND_TRIP_TIME means we have a problem.

// These definitions are copied from PX4 implementation

/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload. We pad the structure ourselves to
/// 32 bit alignment to avoid usage of any pack pragmas.
struct FtpPayload {
	uint16_t	seq_number;	///< sequence number for message
	uint8_t		session;	///< Session id for read and write commands
	uint8_t		opcode;		///< Command opcode
	uint8_t		size;		///< Size of data
	uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
	uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstreadFile - 1: set of burst packets complete, 0: More burst packets coming.
	uint8_t		padding;        ///< 32 bit aligment padding
	uint32_t	offset;		///< Offsets for List and Read commands
	uint8_t		data;		///< command data, varies by Opcode
};

/// @brief Command opcodes
enum Opcode : uint8_t {
	kCmdNone,		///< ignored, always acked
	kCmdTerminateSession,	///< Terminates open Read session
	kCmdResetSessions,	///< Terminates all open Read sessions
	kCmdListDirectory,	///< List files in <path> from <offset>
	kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
	kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
	kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
	kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
	kCmdRemoveFile,		///< Remove file at <path>
	kCmdCreateDirectory,	///< Creates directory at <path>
	kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
	kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
	kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
	kCmdRename,		///< Rename <path1> to <path2>
	kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
	kCmdBurstreadFile,	///< Burst download session file

	kRspAck = 128,		///< Ack response
	kRspNak			///< Nak response
};

/// @brief Error codes returned in Nak response PayloadHeader.data[0].
enum ErrorCode : uint8_t {
	kErrNone,
	kErrFail,			///< Unknown failure
	kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
	kErrInvalidDataSize,		///< PayloadHeader.size is invalid
	kErrInvalidSession,		///< Session is not currently open
	kErrNoSessionsAvailable,	///< All available Sessions in use
	kErrEOF,			///< Offset past end of file for List and Read commands
	kErrRetriesExhausted,
	kErrUnknownCommand		///< Unknown command opcode
};

static const char	kDirentFile = 'F';	///< Identifies File returned from List command
static const char	kDirentDir = 'D';	///< Identifies Directory returned from List command
static const char	kDirentSkip = 'S';	///< Identifies Skipped entry from List command


MavLinkFtpClientImpl::MavLinkFtpClientImpl(int system_id, int component_id)
	: MavLinkNodeImpl(system_id, component_id)
{
}


MavLinkFtpClientImpl::~MavLinkFtpClientImpl()
{
}

bool MavLinkFtpClientImpl::isSupported()
{
	// request capabilities, it will respond with AUTOPILOT_VERSION.
	MavLinkAutopilotVersion ver;
	if (!getCapabilities().wait(5000, &ver)) {
		throw std::runtime_error(Utils::stringf("Five second timeout waiting for response to mavlink command MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES"));
	}
	return (ver.capabilities & static_cast<int>(MAV_PROTOCOL_CAPABILITY::MAV_PROTOCOL_CAPABILITY_FTP)) != 0;
}

void MavLinkFtpClientImpl::subscribe() 
{
	if (subscription_ == 0) {
		subscription_ = getConnection()->subscribe([=](std::shared_ptr<MavLinkConnection> con, const MavLinkMessage& msg) {
			handleResponse(msg);
		});
	}
}

void MavLinkFtpClientImpl::list(MavLinkFtpProgress& progress, const std::string& remotePath, std::vector<MavLinkFileInfo>& files)
{
	if (waiting_) {
		cancel();
	}
	ensureConnection();
	progress_ = &progress;
	files_ = &files;
	command_ = FtpCommandList;
	remote_file_ = remotePath;
	file_index_ = 0;
	runStateMachine();
	progress.complete = true;
	files_ = nullptr;
	progress_ = nullptr;
}

void MavLinkFtpClientImpl::get(MavLinkFtpProgress& progress, const std::string& remotePath, const std::string& localPath)
{
	if (waiting_) {
		cancel();
	}
	ensureConnection();
	progress_ = &progress;
	command_ = FtpCommandGet;
	local_file_ = localPath;
	remote_file_ = remotePath;
	bytes_read_ = 0;
	file_size_ = 0;
	remote_file_open_ = false;

	runStateMachine();
	progress_ = nullptr;
	progress.complete = true;
}

void MavLinkFtpClientImpl::put(MavLinkFtpProgress& progress, const std::string& remotePath, const std::string& localPath)
{
	local_file_ = localPath;
	remote_file_ = remotePath;

	if (waiting_) {
		cancel();
	}
	ensureConnection();
	progress_ = &progress;
	command_ = FtpCommandPut;

	if (openSourceFile()) {
		remote_file_open_ = false;
		runStateMachine();
	}
	progress_ = nullptr;
	progress.complete = true;
}

void MavLinkFtpClientImpl::remove(MavLinkFtpProgress& progress, const std::string& remotePath)
{
	remote_file_ = remotePath;

	if (waiting_) {
		cancel();
	}
	ensureConnection();
	progress_ = &progress;
	command_ = FtpCommandRemove;
	runStateMachine();
	progress_ = nullptr;
	progress.complete = true;
}

void MavLinkFtpClientImpl::runStateMachine()
{
	waiting_ = true;
	retries_ = 0;
	total_time_ = milliseconds::zero();
	messages_ = 0;

	if (remote_file_.size() > 47) {
		throw new std::runtime_error(Utils::stringf("remote file name '%s' is too long, max length is 47", remote_file_.c_str()));
	}

	progress_->cancel = false;
	progress_->error = 0;
	progress_->current = 0;
	progress_->goal = 0;
	progress_->complete = false;
	progress_->longest_delay = 0;
	progress_->message_count = 0;

	int before = 0;
	subscribe();
	nextStep();

	const int monitorInterval = 30; // milliseconds between monitoring progress.
	double rate = 0;
	double totalSleep = 0;

	while (waiting_) 
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(monitorInterval));
		totalSleep += monitorInterval;

		int after = 0;
		{
			std::lock_guard<std::mutex> guard(mutex_);
			after = messages_;
			if (messages_ > 0) {
				rate = static_cast<double>(total_time_.count()) / static_cast<double>(messages_);
				if (progress_ != nullptr)
				{
					progress_->average_rate = rate;
				}
			}
		}

		if (before == after)
		{
			if (rate == 0) {
				rate = MAXIMUM_ROUND_TRIP_TIME;
			}
			if (totalSleep > (rate * TIMEOUT_INTERVAL)) {
				// oops, not getting a response, so retry.
				retry();
				totalSleep = 0;
			}
		}
		else 
		{
			totalSleep = 0;
		}
		before = after;
	}
	waiting_ = false;
}


void MavLinkFtpClientImpl::nextStep()
{
	switch (command_)
	{
	case FtpCommandList:
		listDirectory();
		break;
	case FtpCommandGet:
		readFile();
		break;
	case FtpCommandPut:
		writeFile();
		break;
	case FtpCommandRemove:
		removeFile();
		break;
	default:
		break;
	}
}

void MavLinkFtpClientImpl::removeFile()
{
	MavLinkFileTransferProtocol ftp;
	FtpPayload* payload = reinterpret_cast<FtpPayload*>(&ftp.payload[0]);
	ftp.target_component = getTargetComponentId();
	ftp.target_system = getTargetSystemId();
	payload->opcode = kCmdRemoveFile;
	strncpy(reinterpret_cast<char*>(&payload->data), remote_file_.c_str(), 47);
	payload->size = static_cast<uint8_t>(remote_file_.size());
	sendMessage(ftp);
	recordMessageSent();
}

void MavLinkFtpClientImpl::listDirectory()
{
	MavLinkFileTransferProtocol ftp;
	FtpPayload* payload = reinterpret_cast<FtpPayload*>(&ftp.payload[0]);
	ftp.target_component = getTargetComponentId();
	ftp.target_system = getTargetSystemId();
	payload->opcode = kCmdListDirectory;
	strncpy(reinterpret_cast<char*>(&payload->data), remote_file_.c_str(), 47);
	payload->size = static_cast<uint8_t>(remote_file_.size());
	payload->offset = file_index_;
	sendMessage(ftp);
	recordMessageSent();
}

bool MavLinkFtpClientImpl::openSourceFile()
{
	file_ptr_ = fopen(local_file_.c_str(), "rb");
	if (file_ptr_ == nullptr)
	{
		if (progress_ != nullptr) {
			progress_->error = errno;
			progress_->message = Utils::stringf("Error opening file '%s' for reading, rc=%d", remote_file_.c_str(), progress_->error);
		}
		return false;
	}
	fseek(file_ptr_, 0, SEEK_END);
	long pos = ftell(file_ptr_);
	fseek(file_ptr_, 0, SEEK_SET);
	file_size_ = static_cast<uint64_t>(pos);
	if (progress_ != nullptr) {
		progress_->goal = file_size_;
	}
	return true;
}

bool MavLinkFtpClientImpl::createLocalFile()
{
	if (file_ptr_ == nullptr) {
		auto path = FileSystem::getFullPath(local_file_);
		if (FileSystem::isDirectory(path))
		{
			// user was lazy, only told us where to put the file, so we borrow the name of the file
			// from the source.
			auto remote = FileSystem::getFileName(remote_file_);
			local_file_ = FileSystem::combine(path, remote);
		}
		else
		{
			// check if directory exists.
			FileSystem::removeLeaf(path);
			if (FileSystem::isDirectory(path))
			{
				// perfect.
			}
			else if (FileSystem::exists(path))
			{
				if (progress_ != nullptr) {
					progress_->error = errno;
					progress_->message = Utils::stringf("Error opening file because '%s' is not a directory", path.c_str());
				}
				return false;
			}
			else
			{
				if (progress_ != nullptr) {
					progress_->error = errno;
					progress_->message = Utils::stringf("Error opening file because '%s' should be a directory but it was not found", path.c_str());
				}
				return false;
			}
		}

		file_ptr_ = fopen(local_file_.c_str(), "wb");
		if (file_ptr_ == nullptr)
		{
			if (progress_ != nullptr) {
				progress_->error = errno;
				progress_->message = Utils::stringf("Error opening file '%s' for writing, rc=%d", local_file_.c_str(), errno);
			}
			return false;
		}
		file_size_ = 0;
	}
	return true;
}

void MavLinkFtpClientImpl::readFile()
{
	if (!remote_file_open_) 
	{
		MavLinkFileTransferProtocol ftp;
		FtpPayload* payload = reinterpret_cast<FtpPayload*>(&ftp.payload[0]);
		ftp.target_component = getTargetComponentId();
		ftp.target_system = getTargetSystemId();
		payload->opcode = kCmdOpenFileRO;
		strncpy(reinterpret_cast<char*>(&payload->data), remote_file_.c_str(), 47);
		payload->size = static_cast<uint8_t>(remote_file_.size());
		sendMessage(ftp);
		recordMessageSent();
	}
	else
	{
		if (createLocalFile())
		{
			// use last_message_ so we preserve the sessionid.
			FtpPayload* payload = reinterpret_cast<FtpPayload*>(&last_message_.payload[0]);
			payload->opcode = kCmdReadFile;
			payload->offset = bytes_read_;
			last_message_.target_component = getTargetComponentId();
			last_message_.target_system = getTargetSystemId();
			sendMessage(last_message_);
			recordMessageSent();
			if (progress_ != nullptr) {
				progress_->current = bytes_read_;
			}
		}
		else
		{
			// could not create the local file, so stop.
			waiting_ = false;
		}
	}
}

void MavLinkFtpClientImpl::writeFile()
{

	if (!remote_file_open_) 
	{
		MavLinkFileTransferProtocol ftp;
		FtpPayload* payload = reinterpret_cast<FtpPayload*>(&ftp.payload[0]);
		ftp.target_component = getTargetComponentId();
		ftp.target_system = getTargetSystemId();
		payload->opcode = kCmdOpenFileWO;
		strcpy(reinterpret_cast<char*>(&payload->data), remote_file_.c_str());
		payload->size = static_cast<uint8_t>(remote_file_.size());
		sendMessage(ftp);
		recordMessageSent();
	}
	else
	{
		// must use last_message_ so we preserve the session id.
		MavLinkFileTransferProtocol& ftp = last_message_;
		FtpPayload* payload = reinterpret_cast<FtpPayload*>(&ftp.payload[0]);
		payload->opcode = kCmdWriteFile;
		payload->seq_number = sequence_;
		fseek(file_ptr_, bytes_written_, SEEK_SET);
        uint8_t* data = &payload->data;
		size_t bytes = fread(data, 1, 251 - 12, file_ptr_);
		if (progress_ != nullptr) {
			progress_->current = bytes_written_;
		}
		if (bytes == 0)
		{
			success_ = true;
			reset();
			int err = ferror(file_ptr_);
			if (err != 0) {
				if (progress_ != nullptr) {
					progress_->error = err;
					progress_->message = Utils::stringf("error reading local file, errno=%d", err);
				}
			}
			fclose(file_ptr_);
			file_ptr_ = nullptr;
			waiting_ = false;
		}
		else
		{
			payload->offset = bytes_written_;
			payload->size = static_cast<uint8_t>(bytes);
			ftp.target_component = getTargetComponentId();
			ftp.target_system = getTargetSystemId();
			sendMessage(ftp);
			recordMessageSent();
		}
	}
}

void MavLinkFtpClientImpl::cancel() 
{
	// todo: wait for any pending responses from PX4 so we can safely start a new command.
	reset();
}

void MavLinkFtpClientImpl::close()
{
	if (file_ptr_ != nullptr) {
		reset();
		fclose(file_ptr_);
		file_ptr_ = nullptr;
	}
	if (subscription_ != 0) {
		getConnection()->unsubscribe(subscription_);
		subscription_ = 0;
	}
}

void MavLinkFtpClientImpl::reset()
{
	MavLinkFileTransferProtocol ftp;
	ftp.target_component = getTargetComponentId();
	ftp.target_system = getTargetSystemId();
	FtpPayload* payload = reinterpret_cast<FtpPayload*>(&ftp.payload[0]);
	payload->opcode = kCmdResetSessions;
	sendMessage(ftp);
	recordMessageSent();
}

void MavLinkFtpClientImpl::handleListResponse()
{
	FtpPayload* payload = reinterpret_cast<FtpPayload*>(&last_message_.payload[0]);

	if (payload->offset != file_index_)
	{
		// todo: error handling here? sequence is out of order...
		printf("list got offset %d, but expecting file index %d\n", payload->offset, file_index_);
		retry();
		return;
	}

	if (payload->offset == 0 && payload->size == 0) {
		// directory must be empty then, can't do nextStep because
		// it will just loop for ever re-requesting zero offset into
		// empty directory.

		success_ = true;
		waiting_ = false;
		return;
	}

	// result should be a list of null terminated file names.
    uint8_t* data = &payload->data;
	for (int offset = 0; offset < payload->size; )
	{
		uint8_t dirType = data[offset];
		offset++;
		retries_ = 0;
		file_index_++;
		int len = 0;
		std::string name = reinterpret_cast<char*>(&data[offset]);
		if (dirType == kDirentSkip) {
			// skipping this entry
		}
		else if (dirType == kDirentFile) {
			size_t i = name.find_last_of('\t');
			MavLinkFileInfo info;
			info.is_directory = false;
			len = static_cast<int>(name.size());
			if (i > 0) {
				// remove the file size field.
				std::string size(name.begin() + i + 1, name.end());
				info.size = Utils::to_integer(size);
				name.erase(name.begin() + i, name.end());
			}
			info.name = name;
			//printf("%s\n", name.c_str());
			if (files_ != nullptr) {
				files_->push_back(info);
			}
		}
		else if (dirType == kDirentDir) {
			MavLinkFileInfo info;
			info.is_directory = true;
			info.name = name;
			len = static_cast<int>(name.size());
			if (files_ != nullptr) {
				files_->push_back(info);
			}
		}
		offset += len + 1;
	}
	if (progress_ != nullptr) {
		progress_->current = file_index_;
	}
	// request the next batch.
	nextStep();
}

void MavLinkFtpClientImpl::handleReadResponse()
{
	FtpPayload* payload = reinterpret_cast<FtpPayload*>(&last_message_.payload[0]);
	if (payload->req_opcode == kCmdOpenFileRO) {
		remote_file_open_ = true;
		bytes_read_ = 0;
		retries_ = 0;
		sequence_ = payload->seq_number;
		uint32_t* size = reinterpret_cast<uint32_t*>(&payload->data);
		file_size_ = static_cast<uint64_t>(*size);
		if (progress_ != nullptr) {
			progress_->goal = file_size_;
		}
		nextStep();
	}
	else if (payload->req_opcode == kCmdReadFile) 
	{
		int seq = static_cast<int>(payload->seq_number);
		if (seq != sequence_ + 1)
		{
			printf("packet %d is out of sequence, expecting number %d\n", seq, sequence_ + 1);
			// perhaps this was a late response after we did a retry, so ignore it.
			return;
		}
		else if (file_ptr_ != nullptr)
		{
			sequence_ = payload->seq_number;
			fwrite(&payload->data, payload->size, 1, file_ptr_);
			bytes_read_ += payload->size;
			retries_ = 0;
			nextStep();
		}
	}
}

void MavLinkFtpClientImpl::handleWriteResponse()
{
	FtpPayload* payload = reinterpret_cast<FtpPayload*>(&last_message_.payload[0]);
	if (payload->req_opcode == kCmdOpenFileWO)
	{
		remote_file_open_ = true;
		bytes_written_ = 0;
		retries_ = 0;
		sequence_ = payload->seq_number;
		nextStep();
	}
	else if (payload->req_opcode == kCmdWriteFile)
	{
		int seq = static_cast<int>(payload->seq_number);
		if (seq != sequence_ + 1)
		{
			printf("packet %d is out of sequence, expecting number %d\n", seq, sequence_ + 1);
			// perhaps this was a late response after we did a retry, so ignore it.
			return;
		}

		uint32_t* size = reinterpret_cast<uint32_t*>(&payload->data);
		// payload->size contains the bytes_written from PX4, so that's how much we advance.	
		bytes_written_ += static_cast<int>(*size);
		retries_ = 0;
		sequence_++;
		nextStep();
	}
}

void MavLinkFtpClientImpl::handleRemoveResponse()
{
	success_ = true;
	waiting_ = false;
}

void MavLinkFtpClientImpl::handleResponse(const MavLinkMessage& msg)
{
	if (msg.msgid == static_cast<int>(MavLinkMessageIds::MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL)) {

		last_message_.decode(msg);
		recordMessageReceived();

		FtpPayload* payload = reinterpret_cast<FtpPayload*>(&last_message_.payload[0]);
		if (payload->opcode == kRspNak) {

			// reached the end of the list or the file.
			if (file_ptr_ != nullptr) {
				fclose(file_ptr_);
				file_ptr_ = nullptr;
			}

			int error = static_cast<int>(payload->data);
			if (error == kErrEOF) {
				// end of file or directory listing.
				success_ = true;
				error = 0;
			}
			else 
			{
				success_ = false;
				if (progress_ != nullptr) {
					progress_->error = error;
					progress_->message = Utils::stringf("ftp error %d", error);
				}
			}
			errorCode_ = error;
			waiting_ = false;
			reset();
		}
		else if (payload->opcode == kRspAck) 
		{
			if (progress_ != nullptr) {
				progress_->message_count++;
			}
			// success, data should be following...
			if (payload->req_opcode == kCmdListDirectory)
			{
				handleListResponse();
			}
			else if (payload->req_opcode == kCmdOpenFileRO || payload->req_opcode == kCmdReadFile) {
				handleReadResponse();
			}
			else if (payload->req_opcode == kCmdOpenFileWO || payload->req_opcode == kCmdWriteFile) {
				handleWriteResponse();
			}
			else if (payload->req_opcode == kCmdResetSessions)
			{
				// ack on this cmd is expected
			}
			else if (payload->req_opcode == kCmdRemoveFile)
			{
				handleRemoveResponse();
			}
			else
			{
				// todo: how to handle this? For now we ignore it and let the watchdog kick in and do a retry.
				printf("Unexpected ACK with req_opcode=%d\n", static_cast<int>(payload->req_opcode));
			}
		}
	}
}

void MavLinkFtpClientImpl::MavLinkFtpClientImpl::retry()
{
	retries_++;
	if (retries_ < 10) 
	{
		printf("retry %d\n", retries_);
		nextStep();
	}
	else 
	{
		// give up then.
		errorCode_ = kErrRetriesExhausted;
		success_ = false;
		waiting_ = false;
	}
}

void MavLinkFtpClientImpl::recordMessageSent() 
{
	// tell watchdog we are sending a request
	std::lock_guard<std::mutex> guard(mutex_);
	start_time_ = std::chrono::duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch());
}

void MavLinkFtpClientImpl::recordMessageReceived() 
{
	std::lock_guard<std::mutex> guard(mutex_);
	messages_++;
	milliseconds endTime = std::chrono::duration_cast<milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	milliseconds duration = endTime - start_time_;
	total_time_ += duration;
	if (progress_ != nullptr && duration.count() > progress_->longest_delay)
	{
		progress_->longest_delay = static_cast<double>(duration.count());
	}
}
