#ifndef __CODE_LED_COMM__
#define __CODE_LED_COMM__

#include "CodeLED.h"

class CodeLEDComm
{

protected:
	cv::TickMeter				tick_meter;
	float						led_comm_interval;
	float						led_comm_bitrate;
	float						tracking_threshold;

	CodeLED						code_led;
	std::vector<PointData>		tracking_points;

	void DecodeData();

public:
	void Initialize(Mat in_image);
	void Run(Mat in_image, std::vector<PointData>& out_points);
	void Finalize();
	float GetTime(){	return led_comm_interval;	};
	float GetBitRate() { return led_comm_bitrate; };
};

#endif //__CODE_LED_TRACKER__