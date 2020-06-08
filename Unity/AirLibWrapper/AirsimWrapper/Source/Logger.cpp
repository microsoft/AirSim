#include <time.h>
#include <string>
#include "Logger.h"

#ifdef  _WIN32
  #define WIN32_LEAN_AND_MEAN // combaseapi.h build break fix  
	#include <Windows.h>
	std::ofstream Logger::fileStream;
#elif defined(__linux__) || defined(__APPLE__)
	bfs::ofstream Logger::fileStream;
#endif

Logger* Logger::logger = nullptr; //** This is set to be a nullptr, so are we actually setting it somewhere?


Logger* Logger::GetLogger()
{
	if (logger == nullptr)
	{
		try
		{
			#ifdef _WIN32
				if (CreateDirectoryW(L"Logs", NULL) || GetLastError() == ERROR_ALREADY_EXISTS)
				{
					logger = new Logger();

					// Enabling all LogLevels,
					logger->logLevel_Information = true;
					logger->logLevel_Warning = true;
					logger->logLevel_Error = true;

					time_t now = time(0);
					tm ltm;
					auto err = localtime_s(&ltm, &now);
					char buff[20];
					sprintf_s(buff, "%d%d%d_%d%d", ltm.tm_mday, ltm.tm_mon + 1, ltm.tm_year + 1900, ltm.tm_hour, ltm.tm_min);

					logger->logFileName = "Logs/WrapperDllLog_" + std::string(buff) + ".txt";
					fileStream.open(logger->logFileName, std::ios::out);
				}
            #elif defined(__linux__) || defined(__APPLE__)
				if (bfs::create_directory("Logs") || bfs::exists("Logs"))
				{ 
					logger = new Logger(); //** pointer to logger set here

					// Enabling all LogLevels,
					logger->logLevel_Information = true;
					logger->logLevel_Warning = true;
					logger->logLevel_Error = true;
					time_t now = time(0);
					tm* ltm = localtime(&now);
					auto err = asctime(ltm);
					char buff[20];
					snprintf(buff, 20, "%d%d%d_%d%d", ltm->tm_mday, ltm->tm_mon + 1, ltm->tm_year + 1900, ltm->tm_hour, ltm->tm_min); //** Tested, results are identical to windows
					logger->logFileName = bfs::path{"Logs/WrapperDllLog_" + std::string(buff) + ".txt"};
					fileStream.open(logger->logFileName);
					fileStream << "initial opening";
					
					// delete ltm;


				}
			#endif
		}
		catch (std::exception e)
		{
			#ifdef _WIN32
				throw std::exception(e.what());
			#elif defined(__linux__) || defined(__APPLE__)
				throw std::exception(e);
			#endif
		}
	}
	return logger;
}

Logger::~Logger()
{
	fileStream.flush();
	fileStream.close();
}

std::string Logger::GetCurrentDateTime()
{
	time_t now = time(0);
	#ifdef _WIN32
		char buff[50];
		ctime_s(buff, 50, &now);
	#elif defined(__linux__) || defined(__APPLE__)
		auto buff = asctime(localtime(&now));
	#endif
	
	return std::string(buff).substr(0, std::string(buff).size() - 1);
}

void Logger::SetLogLevel(LogLevel level, bool status)
{
	if (level == LogLevel::Information)
		logLevel_Information = status;
	else if (level == LogLevel::Warning)
		logLevel_Warning = status;
	else if (level == LogLevel::Error)
		logLevel_Error = status;
}

void Logger::WriteLog(const std::string log, LogLevel level)
{
	if (logLevel_Information && level == LogLevel::Information)
	{
		fileStream << "\n[Information] \t<" << GetCurrentDateTime() << ">" << " \t" << log;
	}
	else if (logLevel_Warning && level == LogLevel::Warning)
	{
		fileStream << "\n[Warnning] \t\t<" << GetCurrentDateTime() << ">" << " \t" << log;

	}
	else if (logLevel_Error && level == LogLevel::Error)
	{
		fileStream << "\n[Error] \t\t<" << GetCurrentDateTime() << ">" << " \t" << log;
	}
	fileStream.flush();
}
