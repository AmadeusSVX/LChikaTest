#ifndef __PGR_FREA3__
#define __PGR_FREA3__
#include <opencv2\opencv.hpp>
#include "FlyCapture2.h"
#include <thread>
#include <mutex>

using namespace cv;
using namespace FlyCapture2;


class Flea3
{
private:
	bool	is_init;
	Mat		capture_mat;
	Mat		thread_mat;

	// for FlyCapture SDK
	Camera	cam;
	Image	raw_image;
	Image	gray_image;

	std::thread _thread;
	std::mutex	_mutex;

public:
	Flea3(unsigned int in_id);
	~Flea3();
	
	Mat Run();

	void SetShutter(float in_shutter);
	void SetExposure(float in_exposure);
	void SetGain(float in_gain);
	void SetGamma(float in_gamma);
};

#endif // __PGR_FREA3__