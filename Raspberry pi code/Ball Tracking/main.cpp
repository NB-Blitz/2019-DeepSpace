/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"

using namespace cv;
using namespace std;

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

static const char* configFile = "/home/pi/Blitz/CameraConfigs/blitz.json";

namespace 
{

bool Run = true;


unsigned int team;
bool server = false;

struct CameraConfig 
{
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

std::vector<CameraConfig> cameraConfigs;

wpi::raw_ostream& ParseError() 
{
  return wpi::errs() << "config error in '" << configFile << "': ";
}

bool ReadCameraConfig(const wpi::json& config) 
{
  CameraConfig c;

  // name
  try 
  {
    c.name = config.at("name").get<std::string>();
  }
  catch (const wpi::json::exception& e) 
  {
    ParseError() << "could not read camera name: " << e.what() << '\n';
    return false;
  }

  // path
  try 
  {
    c.path = config.at("path").get<std::string>();
  } 
  catch (const wpi::json::exception& e) 
  {
    ParseError() << "camera '" << c.name << "': could not read path: " << e.what() << '\n';
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() 
{
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) 
  {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message() << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try 
  {
    j = wpi::json::parse(is);
  } 
  catch (const wpi::json::parse_error& e) 
  {
    ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
    return false;
  }

  // top level must be an object
  if (!j.is_object()) 
  {
    ParseError() << "must be JSON object\n";
    return false;
  }

  // team number
  try 
  {
    team = j.at("team").get<unsigned int>();
  } 
  catch (const wpi::json::exception& e) 
  {
    ParseError() << "could not read team number: " << e.what() << '\n';
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) 
  {
    try 
	{
      auto str = j.at("ntmode").get<std::string>();
      wpi::StringRef s(str);
      if (s.equals_lower("client")) 
	  {
        server = false;
      } 
	  else if (s.equals_lower("server")) 
	  {
        server = true;
      }
	  else 
	  {
        ParseError() << "could not understand ntmode value '" << str << "'\n";
      }
    } 
	catch (const wpi::json::exception& e)
	{
      ParseError() << "could not read ntmode: " << e.what() << '\n';
    }
  }

  // cameras
  try
  {
    for (auto&& camera : j.at("cameras"))
	{
      if (!ReadCameraConfig(camera)) return false;
    }
  }
  catch (const wpi::json::exception& e) 
  {
    ParseError() << "could not read cameras: " << e.what() << '\n';
    return false;
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config)
 {
  wpi::outs() << "Starting camera '" << config.name << "' on " << config.path << '\n';
  auto inst = frc::CameraServer::GetInstance();
  cs::UsbCamera camera{config.name, config.path};
  auto server = inst->StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
  {
    server.SetConfigJson(config.streamConfig);
  }

  return camera;
}
}


int main(int argc, char* argv[]) 
{
  if (argc >= 2)
  {
	  configFile = argv[1];
  }

  // read configuration
  if (!ReadConfig())
  {
	  return EXIT_FAILURE;
  }

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  auto table = ntinst.GetTable("datatable");
  auto SmartDashboard = ntinst.GetTable("SmartDashboard");
  
  nt::NetworkTableEntry lowHEntry = table->GetEntry("LowHValue");
  nt::NetworkTableEntry lowSEntry = table->GetEntry("LowSValue");
  nt::NetworkTableEntry lowVEntry = table->GetEntry("LowVValue");
  
  nt::NetworkTableEntry highHEntry = table->GetEntry("HighHValue");
  nt::NetworkTableEntry highSEntry = table->GetEntry("HighSValue");
  nt::NetworkTableEntry highVEntry = table->GetEntry("HighVValue");
  
  nt::NetworkTableEntry cameraSelectionEntry = table->GetEntry("Camera Num");
  
  nt::NetworkTableEntry xOffsetEntry = SmartDashboard->GetEntry("XOffset");
  nt::NetworkTableEntry yOffsetEntry = SmartDashboard->GetEntry("YOffset");
  nt::NetworkTableEntry DistanceEntry = SmartDashboard->GetEntry("Distance");
  
  
  if (server)
  {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } 
  else 
  {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
  }

  // start cameras
  std::vector<cs::VideoSource> cameras;
  std::vector<cs::CvSink> cameraSinks;
  cs::CvSink cameraSink;
  for (auto&& cameraConfig : cameraConfigs)
  {
    cameras.emplace_back(StartCamera(cameraConfig));
	//cameraSinks.emplace_back(frc::CameraServer::GetInstance()->GetVideo(cameraConfig.name));
	cameraSink = frc::CameraServer::GetInstance()->GetVideo(cameraConfig.name);
  }
  
  cs::VideoMode cameraMode = cameras[0].GetVideoMode();
  
  cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Main Camera", cameraMode.width, cameraMode.height);
  
  int CameraCenterX = cameraMode.width/2;
  int CameraCenterY = cameraMode.height/2;
  
  lowHEntry.SetDouble(0);
  lowSEntry.SetDouble(139);
  lowVEntry.SetDouble(170);
  
  
  highHEntry.SetDouble(14);
  highSEntry.SetDouble(255);
  highVEntry.SetDouble(255);
  
  cameraSelectionEntry.SetDouble(0);

  while(Run)
  {
	  int camera = 0;
	  
	  Mat originalImg;
	  
	  double lowHValue = lowHEntry.GetDouble(0);
	  double lowSValue = lowSEntry.GetDouble(0);
	  double lowVValue = lowVEntry.GetDouble(0);
	  
	  double highHValue = highHEntry.GetDouble(255);
	  double highSValue = highSEntry.GetDouble(255);
	  double highVValue = highVEntry.GetDouble(255);
	  
	  camera = cameraSelectionEntry.GetDouble(0);
	  
	  cameraSink.GrabFrame(originalImg);
	  
	  
	  if(!originalImg.empty())
	  {
	    Mat HSVImg;
		cvtColor(originalImg, HSVImg, COLOR_BGR2HSV);
	  
	  
	    Mat thresholdImg;
	    inRange(HSVImg, Scalar(lowHValue, lowSValue, lowVValue), Scalar(highHValue, highSValue, highVValue), thresholdImg);
		
		vector<vector<Point>> contours;
		vector<Point> fieldContour;
		vector<Vec4i> heirarchy;
		
		findContours(thresholdImg, contours, heirarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		
		if(contours.size() > 0)
		{
			fieldContour = contours[0];
			
			for(int i = 0; i < contours.size(); i++)
			{
				if(contours[i].size() > 2)
				{
					if(contourArea(contours[i]) > contourArea(fieldContour))
					{
						fieldContour = contours[i];
					}
				}
			}
			
			Point2f center;
			float radius;

			minEnclosingCircle(fieldContour, center, radius);
			
			circle(originalImg, center, radius, Scalar(0, 255, 0), 2);
			circle(originalImg, center, 2, Scalar(255, 0, 0), 2);
			
			Point2f CameraCenter = Point2f(CameraCenterX, CameraCenterY);
			
			circle(originalImg, CameraCenter, 2, Scalar(0, 0, 255), 2);
			
			double xCenterDist = CameraCenterX - center.x;
			double yCenterDist = CameraCenterY - center.y;
			
			double Distance = (13 * 369.23)/(2 * radius);
			
			
			if(contourArea(fieldContour) <  500)
			{
				xCenterDist = 0;
				yCenterDist = 0;
				Distance = 0;
			}
			
			xOffsetEntry.SetDouble(xCenterDist);
			yOffsetEntry.SetDouble(yCenterDist);
			
			DistanceEntry.SetDouble(Distance);
			
		}
		
		if(camera == 0)
		{
			outputStream.PutFrame(thresholdImg);
		} 
		else if(camera == 1)
		{
			outputStream.PutFrame(originalImg);
		} 
		
		
		
	  }
  }
}

