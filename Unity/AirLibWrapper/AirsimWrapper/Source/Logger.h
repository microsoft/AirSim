#ifndef LOGGER_FILE
#define LOGGER_FILE

#include <fstream>
#if defined(__linux__) || defined(__APPLE__)
	#include <boost/filesystem.hpp>
	#include <boost/filesystem/fstream.hpp>
	namespace bfs = boost::filesystem;
#endif

#define LOGGER Logger::GetLogger()

class Logger
{
public:
	enum LogLevel {
		Information,
		Warning,
		Error
	};

private:
	Logger() {}
	Logger(Logger const&) = delete;
	void operator=(Logger const&) = delete;
	std::string GetCurrentDateTime();

	bool logLevel_Information;
	bool logLevel_Warning;
	bool logLevel_Error;

	static Logger* logger;
	#ifdef _WIN32
		static std::ofstream fileStream;
		std::string logFileName;
	#else
		static 	bfs::ofstream fileStream;
				bfs::path 	logFileName;
	#endif
	

public:
	~Logger();
	static Logger* GetLogger();
	void WriteLog(const std::string log, LogLevel type = LogLevel::Information);
	void SetLogLevel(LogLevel level, bool status);
};

#endif // !LOGGER_FILE