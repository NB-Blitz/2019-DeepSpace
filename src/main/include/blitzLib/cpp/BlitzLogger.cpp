/*
 * BlitzLogger.cpp
 *
 *  Created on: Aug 5, 2017
 *      Author: Sam
 */
#include <BlitzLogger.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>


/* Constructor for BlitzLogger Class
 *
 * Logs everything from your input and below
 *
 * @parameter: logLevel
 *
 * Error = 0,
 * Warning = 1,
 * Info = 2,
 * Debug = 3,
 * Trace = 4
 */
Blitz::BlitzLogger::BlitzLogger(int logLevel)
{
	this->logLevel = logLevel;
}

void Blitz::BlitzLogger::init()
{
	std::string dateTime;
	dateTime = "/home/lvuser/";
	dateTime.append(spaceToUnderscore(getTimeStamp()));
	dateTime.append(".txt");
	logfile = fopen(dateTime.c_str(), "a");
}

std::string Blitz::BlitzLogger::spaceToUnderscore(std::string text)
{
	for(int i = 0; i < text.length(); i++)
	{
		if(text[i] == ' ')
		{
			text[i] = '_';
		}
		else if(text[i] == ':')
		{
			text[i] = '_';
		}
	}
	return text;
}

void Blitz::BlitzLogger::close()
{
	fflush(logfile);
	fclose(logfile);
}

std::string Blitz::BlitzLogger::getTimeStamp()
{
	std::chrono::system_clock::time_point p = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(p);
	std::string timeStamp = ctime(&t);
	timeStamp = timeStamp.substr(0,timeStamp.length()-1);
	return timeStamp;
}

std::string Blitz::BlitzLogger::getTicksElapsed()
{

}

void Blitz::BlitzLogger::log(std::string currentStage, std::string level, std::string message)
{
	std::string timeStamp = Blitz::BlitzLogger::getTimeStamp();
	if(logfile != NULL)
	{
		fputs("[", logfile);
		fputs(timeStamp.c_str(), logfile);//writes timestamp to file
		fputs("]", logfile);
		fputs("", logfile); //Separating time stamp and message type
		fputs("[", logfile);
		fputs(currentStage.c_str(), logfile); //writes current stage to file
		fputs("]", logfile);
		fputs("", logfile); //Separating time stamp and message type
		fputs("[", logfile);
		fputs(level.c_str(), logfile); //writes log level to file
		fputs("]", logfile);
		fputs("", logfile); //Separating message type and message
		fputs(message.c_str(), logfile); //Writes message to file
		fputc('\r\n', logfile); //Sets writing location to next line for next log message
	}
}

void Blitz::BlitzLogger::error(std::string currentStage, std::string message)
{
	if(logLevel >= Error)
	{
		log(currentStage, "Error", message);
	}
}

void Blitz::BlitzLogger::warning(std::string currentStage, std::string message)
{
	if(logLevel >= Warning)
	{
		log(currentStage, "Warning", message);
	}
}

void Blitz::BlitzLogger::info(std::string currentStage, std::string message)
{
	if(logLevel >= Info)
	{
		log(currentStage, "Info", message);
	}
}

void Blitz::BlitzLogger::debug(std::string currentStage, std::string message)
{
	if(logLevel >= Debug)
	{
		log(currentStage, "Debug", message);
	}
}

void Blitz::BlitzLogger::trace(std::string currentStage, std::string message)
{
	if(logLevel >= Trace)
	{
		log(currentStage, "Trace", message);
	}
}
