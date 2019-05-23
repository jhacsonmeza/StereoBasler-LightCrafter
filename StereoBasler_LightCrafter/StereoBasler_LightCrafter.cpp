#include <pylon/PylonIncludes.h>
#include <pylon/usb/BaslerUsbInstantCameraArray.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <filesystem>
#include <stdexcept>

#include "LightCrafter/LC_Flash.h"

using namespace Pylon;
using namespace cv;
using namespace std;
using namespace std::filesystem;


int main(int argc, char* argv[])
{
	// Root path to store images
	path root = "F:\\StereoBasler_LightCrafter\\acquisition\\";

	if (!is_directory(root))
		if (!create_directory(root))
			return -1;

	// If there are no paths to each source, create them
	if (!is_directory(root / "L"))
		if (!create_directory(root / "L"))
			return -1;

	if (!is_directory(root / "R"))
		if (!create_directory(root / "R"))
			return -1;


	// The exit code of the sample application.
	int exitCode = 0;

	// Before using any pylon methods, the pylon runtime must be initialized. 
	PylonInitialize();

	try
	{
		// Get the transport layer factory.
		CTlFactory& tlFactory = CTlFactory::GetInstance();

		// Get all attached devices and exit application if no device is found.
		DeviceInfoList_t devices;
		if (tlFactory.EnumerateDevices(devices) == 0)
			throw RUNTIME_EXCEPTION("No camera present.");

		// Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
		CBaslerUsbInstantCameraArray cameras(2); //Equivalent to: CInstantCameraArray cameras(2); but for usb cameras Basler_UsbCameraParams::

		// Create and attach all Pylon Devices.
		for (size_t i = 0; i < cameras.GetSize(); ++i)
		{
			cameras[i].Attach(tlFactory.CreateDevice(devices[i]));

			cameras[i].Open();

			cameras[i].LineSelector.SetValue(Basler_UsbCameraParams::LineSelector_Line1);
			cameras[i].LineMode.SetValue(Basler_UsbCameraParams::LineMode_Input);

			cameras[i].AcquisitionMode.SetValue(Basler_UsbCameraParams::AcquisitionMode_Continuous); //AcquisitionMode_SingleFrame - AcquisitionMode_Continuous

			//cameras[i].TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameBurstStart);
			//cameras[i].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_Off);
			//cameras[i].TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
			cameras[i].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_Off);

			cameras[i].AcquisitionFrameRateEnable.SetValue(false);
			cameras[i].TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
			cameras[i].TriggerActivation.SetValue(Basler_UsbCameraParams::TriggerActivation_RisingEdge);

			cameras[i].ExposureMode.SetValue(Basler_UsbCameraParams::ExposureMode_Timed);
			cameras[i].ExposureAuto.SetValue(Basler_UsbCameraParams::ExposureAuto_Off);
			cameras[i].ExposureTime.SetValue(2000);
			cameras[i].TriggerDelay.SetValue(0);
			cameras[i].SensorReadoutMode.SetValue(Basler_UsbCameraParams::SensorReadoutMode_Fast);

			//cameras[i].Close();

			// Print the model name of the camera.
			cout << "Using device " << cameras[i].GetDeviceInfo().GetModelName() << endl;
		}
		cout << endl;


		// Variables to use
		int iR, iL; // Index of cameras
		CGrabResultPtr ptrGrabResultL, ptrGrabResultR; // Store retrieve result as pointer of both cameras

		int cntImagesNum = 0; // Initialize counter of images to store them with index number
		string strFileName; // Filename string of images to store

		CPylonImage imgLeft, imgRight; // pylon images
		Mat imL, imR, imLrs, imRrs, cat; // OpenCV matrices
		vector<Mat> matrices; // vector of Mat for image concatenation
		CImageFormatConverter formatConverter;


		// Check which camera is R and which L to assign the correct camera index
		for (int i = 0; i < cameras.GetSize(); i++)
		{
			if (cameras[i].GetDeviceInfo().GetSerialNumber() == "22151646")
				iR = i;
			else if (cameras[i].GetDeviceInfo().GetSerialNumber() == "21953150")
				iL = i;
		}


		// Set up format convert to store pylon image as grayscale
		formatConverter.OutputPixelFormat = PixelType_Mono8;
		// Set up window to show acquisition
		namedWindow("Acquisition", WINDOW_NORMAL); resizeWindow("Acquisition", 620 * 2, 480);
		// Start grabbing cameras
		cameras.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly, Pylon::GrabLoop_ProvidedByUser);


		bool capt = 0;
		while (cameras.IsGrabbing())
		{
			// Basler frame capture
			cameras[iL].RetrieveResult(INFINITE, ptrGrabResultL, TimeoutHandling_ThrowException);
			cameras[iR].RetrieveResult(INFINITE, ptrGrabResultR, TimeoutHandling_ThrowException);
			cout << "Status: " << cameras[0].AcquisitionStatus() << endl;


			// If the image was grabbed successfully.
			if (ptrGrabResultL->GrabSucceeded() && ptrGrabResultR->GrabSucceeded())
			{
				// Convet left image to pylon image and then to Mat
				formatConverter.Convert(imgLeft, ptrGrabResultL);
				imL = Mat(ptrGrabResultL->GetHeight(), ptrGrabResultL->GetWidth(), CV_8UC1, (uint8_t *)imgLeft.GetBuffer());

				// Convet right image to pylon image and then to Mat
				formatConverter.Convert(imgRight, ptrGrabResultR);
				imR = Mat(ptrGrabResultR->GetHeight(), ptrGrabResultR->GetWidth(), CV_8UC1, (uint8_t *)imgRight.GetBuffer());


				// Resize basler and US images for visualization purposes
				resize(imL, imLrs, Size(620, 480));
				resize(imR, imRrs, Size(620, 480));

				// Concatenate three images
				hconcat(imLrs, imRrs, cat);

				// show images
				imshow("Acquisition", cat);
				char c = waitKey(1);

				if (c == 27)
					break;
				else if (c == 'c')
				{

					capt = 1;

					cameras[0].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
					cameras[1].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);

					if (LightCrafterFlash(150000, 150000, 0, "0-1-2") < 0) // LightCrafterFlash(120000, 120000, 0, "0-1-2") LightCrafterFlash(400000, 400000, 0, "0-1-2")
						return -1;
				}
			}

			if (ptrGrabResultL->GrabSucceeded() && ptrGrabResultR->GrabSucceeded() && capt)
			{

				// Convet left image to pylon image and then to Mat
				formatConverter.Convert(imgLeft, ptrGrabResultL);
				imL = Mat(ptrGrabResultL->GetHeight(), ptrGrabResultL->GetWidth(), CV_8UC1, (uint8_t *)imgLeft.GetBuffer());

				// Convet right image to pylon image and then to Mat
				formatConverter.Convert(imgRight, ptrGrabResultR);
				imR = Mat(ptrGrabResultR->GetHeight(), ptrGrabResultR->GetWidth(), CV_8UC1, (uint8_t *)imgRight.GetBuffer());


				strFileName = root.string() + "L\\left" + to_string(cntImagesNum) + ".bmp";
				imwrite(strFileName, imL);

				strFileName = root.string() + "R\\right" + to_string(cntImagesNum) + ".bmp";
				imwrite(strFileName, imR);

				cout << "+Images with index " << cntImagesNum << " has been collected" << endl;

				cntImagesNum++;
				if (cntImagesNum > 6)
				{
					cntImagesNum = 0;
					capt = 0;
					cameras[0].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_Off);
					cameras[1].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_Off);
				}


				// Resize basler and US images for visualization purposes
				resize(imL, imLrs, Size(620, 480));
				resize(imR, imRrs, Size(620, 480));

				// Concatenate three images
				hconcat(imLrs, imRrs, cat);

				// show images
				imshow("Acquisition", cat);
				char c = waitKey(1);

				if (c == 27)
					break;

			}
		}

		//cameras[0].Close();
		//cameras[1].Close();
		cameras.Close();
		destroyAllWindows();

	}
	catch (const GenericException &e)
	{
		// Error handling
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
		exitCode = 1;
	}

	// Comment the following two lines to disable waiting on exit.
	cerr << endl << "Press Enter to exit." << endl;
	while (cin.get() != '\n');

	// Releases all pylon resources. 
	PylonTerminate();

	return exitCode;
}

