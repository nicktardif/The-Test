#include "stdafx.h"

using namespace cv;
using namespace std;

// NOTE: This code was modified from http://tuxvoid.blogspot.com/2014/03/kinect-v2-developer-preview-opencv-248_16.html

inline void CHECKERROR(HRESULT n) {
	if (!SUCCEEDED(n)) {
		std::stringstream ss;
		ss << "ERROR " << std::hex << n << std::endl;
		std::cin.ignore();
		std::cin.get();
		throw std::runtime_error(ss.str().c_str());
	}
}

// Safe release for interfaces

/* For some reason this doesn't compile
template
inline void SAFERELEASE(Interface *& pInterfaceToRelease) {
if (pInterfaceToRelease != nullptr) {
pInterfaceToRelease->Release();
pInterfaceToRelease = nullptr;
}
}
*/

IDepthFrameReader* depthFrameReader = nullptr; // depth reader
int depthHeight	= 424;
int depthWidth	= 512;
int depthPixelCount = depthHeight * depthWidth;

void processIncomingData() {
	IDepthFrame *data = nullptr;
	IFrameDescription *frameDesc = nullptr;
	HRESULT hr = -1;
	UINT16 *depthBuffer = nullptr;
	USHORT nDepthMinReliableDistance = 0;
	USHORT nDepthMaxReliableDistance = 0;

	hr = depthFrameReader->AcquireLatestFrame(&data);
	if (SUCCEEDED(hr)) hr = data->get_FrameDescription(&frameDesc);
	if (SUCCEEDED(hr)) hr = data->get_DepthMinReliableDistance(
		&nDepthMinReliableDistance);
	if (SUCCEEDED(hr)) hr = data->get_DepthMaxReliableDistance(
		&nDepthMaxReliableDistance);

	if (SUCCEEDED(hr)) {
		if (SUCCEEDED(frameDesc->get_Height(&depthHeight)) &&
			SUCCEEDED(frameDesc->get_Width(&depthWidth))) {
			depthBuffer = new UINT16[depthPixelCount];
			hr = data->CopyFrameDataToArray(depthHeight * depthWidth, depthBuffer);
			if (SUCCEEDED(hr)) {
				cv::Mat depthMap = cv::Mat(depthHeight, depthWidth, CV_16U, depthBuffer);
				cv::Mat img0 = cv::Mat::zeros(depthHeight, depthWidth, CV_8UC1);
				cv::Mat img1;
				double scale = 255.0 / (nDepthMaxReliableDistance -
					nDepthMinReliableDistance);
				depthMap.convertTo(img0, CV_8UC1, scale);
				applyColorMap(img0, img1, cv::COLORMAP_JET);
				cv::imshow("Depth Window", img1);
			}
		}
	}
	if (depthBuffer != nullptr) {
		delete[] depthBuffer;
		depthBuffer = nullptr;
	}

	if (data != nullptr) {
		data->Release();
		data = nullptr;
	}
}

int main(int argc, char** argv) {
	HRESULT hr;
	IKinectSensor* kinectSensor = nullptr;     // kinect sensor

	// initialize Kinect Sensor
	hr = GetDefaultKinectSensor(&kinectSensor);
	if (FAILED(hr) || !kinectSensor) {
		std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
		return -1;
	}
	CHECKERROR(kinectSensor->Open());

	// initialize depth frame reader
	IDepthFrameSource* depthFrameSource = nullptr;
	CHECKERROR(kinectSensor->get_DepthFrameSource(&depthFrameSource));
	CHECKERROR(depthFrameSource->OpenReader(&depthFrameReader));

	if (depthFrameSource != nullptr) {
		depthFrameSource->Release();
		depthFrameSource = nullptr;
	}

	namedWindow("Depth Window", WINDOW_AUTOSIZE);

	// Initialize performance measuring variables
	int frameCount = 0;
	int performancePeriod = 30;
	int startProcessingTime = INFINITY;
	int endProcessingTime;
	float avgProcessingTime = 0.0; 
	float processingTimeSeconds, avgFPS;

	while (depthFrameReader) {

		// Print out the performance and reset the frameCount
		if (frameCount % performancePeriod == 0) {
			endProcessingTime = clock();
			processingTimeSeconds = float(endProcessingTime - startProcessingTime) / CLOCKS_PER_SEC;
			startProcessingTime = clock();

			printf("Average FPS over the last %d frames: %f seconds\n", performancePeriod, float(performancePeriod / processingTimeSeconds));

			frameCount = 0;
		}

		frameCount++;

		processIncomingData();

		int key = cv::waitKey(10);
		if (key == 'q'){
			break;
		}
	}

	// de-initialize Kinect Sensor
	CHECKERROR(kinectSensor->Close());

	if (kinectSensor != nullptr) {
		kinectSensor->Release();
		kinectSensor = nullptr;
	}
	return 0;
}