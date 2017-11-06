#ifndef __CODE_LED_TRACKER__
#define __CODE_LED_TRACKER__

#include "CodeLED.h"

class CodeLEDTracker
{

protected:
	float					tracking_threshold;

	CodeLED					code_led;
	std::vector<PointData>	tracking_points;

public:
	void Initialize(Mat in_image);
	void Run(Mat in_image, std::vector<PointData>& out_points);
	void Finalize();
};

#endif //__CODE_LED_TRACKER__