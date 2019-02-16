// WPiLib
#include "cscore.h"
#include "cameraserver/CameraServer.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
// OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
// CPP General Libraries
#include <iostream>
#include <thread>

// Namespaces
using namespace std; // Standard
using namespace cv;  // OpenCV
using namespace nt; // Network Tables
using namespace cs; // Camera Server
using namespace frc; // FRC

// Constant Variables
const std::string TABLE = "BlitzTableOfDoomTM";

const double SPEED = 0.5; // Speed of the motors move.
const double SPEED_R = 0.2;

const double SPEED_H = 0.4;
const double DEAD_ZONE = 0.1; // Horizontal Dead Zone (Measured in Percent)

const double DEAD_ZONE_R = 5; // Rotational Dead Zone (Measured in Degreed)
const int SENSITIVITY_LOW = 180;
const int SENSITIVITY_HIGH = 255;

int main()
{
	// Variables
	double alignX = 0;
	double alignZ = 0;
	double lastZ = 0;
	
	UsbCamera cam{ "CamA", 0 };
	CvSink cvSink{ "Sink" };
	//CvSource outputStreamStdA{ "Out", VideoMode::kBGR, 640, 480, 30 };
	//CvSource outputStreamStdB{ "Img", VideoMode::kBGR, 640, 480, 30 };
	CvSource outputStreamStdA = CameraServer::GetInstance()->PutVideo("Out", 640, 480);
	CvSource outputStreamStdB = CameraServer::GetInstance()->PutVideo("Cam", 640, 480);
	
	
	vector<vector<Point> > contours;
	auto inst = NetworkTableInstance::GetDefault();
	inst.StartClientTeam(5148);
	auto table = inst.GetTable(TABLE);
	NetworkTableEntry netAlignX = table->GetEntry("alignX");
	NetworkTableEntry netAlignZ = table->GetEntry("alignZ");

	// Initialization
	cam.SetBrightness(50);
	cam.SetExposureManual(35);
	cvSink.SetSource(cam);
	cvSink.SetEnabled(true);
	

	// Loop while camera is connected.
	while (cam.IsConnected() && cam.IsEnabled())
	{
		// Variables
		Mat frame;
		Mat out;
		Point center;
		
		// Get and Process camera data.
		cvSink.GrabFrame(frame);
		if (frame.empty())
			continue;
		center = Point(frame.size().width / 2, frame.size().height / 2);
		out = frame;
		cvtColor(frame, frame, cv::COLOR_BGR2RGB);

		inRange(frame, Scalar(SENSITIVITY_LOW, SENSITIVITY_LOW, SENSITIVITY_LOW), Scalar(SENSITIVITY_HIGH, SENSITIVITY_HIGH, SENSITIVITY_HIGH), frame);
		findContours(frame, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		// Find and draw the largest contour.
		if (contours.size() > 0)
		{
			std::vector<cv::Point, std::allocator<cv::Point>> contour = contours.at(0);
			for (int i = 0; i < contours.size(); i++)
			{
				if (contourArea(contours.at(i), false) > contourArea(contour, false))
				{
					contour = contours.at(i);
				}
			}

			Point2f vtx[4];
			RotatedRect box = minAreaRect(contour);
			box.points(vtx);

			for (int i = 0; i < 4; i++)
				line(out, vtx[i], vtx[(i + 1) % 4], Scalar(0, 255, 0), 2);
			drawMarker(out, box.center, Scalar(0, 255, 0), 0, 20, 5, 8);

			alignX = ((double)(box.center.x - center.x) / center.x);
			alignZ = box.angle;
			

			if (alignZ < -45)
			{
				alignZ += 90;
			}
			alignZ *= -1;
			/*
			if (alignZ > 0)
				alignZ = 45 - alignZ;
			else
			{
				alignZ = 45 + alignZ;
				alignZ *= -1;
			}*/

			putText(out, to_string(alignX), box.center, FONT_HERSHEY_COMPLEX_SMALL, 3, Scalar(255, 0, 0), 3);
			line(out, box.center, center, Scalar(0, 255, 0), 5);
		}

		// Output Data
		outputStreamStdA.PutFrame(frame);
		outputStreamStdB.PutFrame(out);

		netAlignX.SetDouble(alignX);
		netAlignZ.SetDouble(alignZ);

		cout << "alignX: " << alignX << endl;
		cout << "alignZ: " << alignZ << endl;
	}

}
