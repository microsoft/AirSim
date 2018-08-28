#ifndef LOGGER_FILE
#define LOGGER_FILE

#include <iostream>
#include <fstream>

class Logger
{
private:
	Logger(Logger const&) = delete;
	void operator=(Logger const&) = delete;

public:
	enum LogType {
		Information,
		Warnning,
		Error
	};
	static bool firstTimeFileAccess;

public:
	static void WriteLogLine(std::string logLine, LogType type)
	{
		std::ofstream file;
		if (firstTimeFileAccess)
		{
			file.open("WrapperDllLog.txt", std::ios::out);
			firstTimeFileAccess = false;
		}
		else
		{
			file.open("WrapperDllLog.txt", std::ios::app);
		}

		std::string typeString = "Information";
		if (type == LogType::Error)
			typeString = "Error";
		else if (type == LogType::Warnning)
			typeString = "Warnning";

		file << typeString.c_str() << " -\t";
		file << logLine.c_str() << "\n";
		file.flush();
		file.close();
	}
};

#endif // !LOGGER_FILE