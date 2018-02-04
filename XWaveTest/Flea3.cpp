#include "Flea3.h"

Flea3::Flea3(unsigned int in_id)
{
	BusManager bus_mgr;
	unsigned int num_camera;

	is_init = false;

	auto error = bus_mgr.GetNumOfCameras(&num_camera);

	if (error != PGRERROR_OK || num_camera < in_id) { return; }

	PGRGuid guid;
	error = bus_mgr.GetCameraFromIndex(in_id, &guid);
	if (error != PGRERROR_OK) { return; }

	// Connect to a camera
	error = cam.Connect(&guid);
	if (error != PGRERROR_OK) { return; }


	// Start capturing images
	error = cam.StartCapture();
	if (error != PGRERROR_OK) { return; }

	cam.RetrieveBuffer(&raw_image);

	raw_image.Convert(PixelFormat::PIXEL_FORMAT_MONO8, &gray_image);

	thread_mat = Mat(gray_image.GetRows(), gray_image.GetCols(), CV_8U);
	capture_mat = Mat(gray_image.GetRows(), gray_image.GetCols(), CV_8U);

	is_init = true;

	// thread start
	_thread = std::thread([&] 
	{
		while (is_init)
		{
			cam.RetrieveBuffer(&raw_image);
			raw_image.Convert(PixelFormat::PIXEL_FORMAT_MONO8, &gray_image);
			std::lock_guard<std::mutex> lock(_mutex);
			thread_mat.data = raw_image.GetData();
		}
	});

}

Flea3::~Flea3()
{
	is_init = false;
	_thread.join();

	cam.Disconnect();
}

void Flea3::SetShutter(float in_shutter)
{
	Property in_property;
	in_property.absControl = true;
	in_property.absValue = in_shutter;
	in_property.onOff = true;
	in_property.autoManualMode = false;
	in_property.type = PropertyType::SHUTTER;

	std::lock_guard<std::mutex> lock(_mutex);
	cam.SetProperty(&in_property);
}

void Flea3::SetExposure(float in_exposure)
{
	Property in_property;
	in_property.absControl = true;
	in_property.absValue = in_exposure;
	in_property.onOff = true;
	in_property.autoManualMode = false;
	in_property.type = PropertyType::AUTO_EXPOSURE;

	std::lock_guard<std::mutex> lock(_mutex);
	cam.SetProperty(&in_property);
}

void Flea3::SetGain(float in_gain)
{
	Property in_property;
	in_property.absControl = true;
	in_property.absValue = in_gain;
	in_property.onOff = true;
	in_property.autoManualMode = false;
	in_property.type = PropertyType::GAIN;

	std::lock_guard<std::mutex> lock(_mutex);
	cam.SetProperty(&in_property);
}

void Flea3::SetGamma(float in_gamma)
{
	Property in_property;
	in_property.absControl = true;
	in_property.absValue = in_gamma;
	in_property.onOff = true;
	in_property.autoManualMode = false;
	in_property.type = PropertyType::GAMMA;

	std::lock_guard<std::mutex> lock(_mutex);
	cam.SetProperty(&in_property);
}

Mat Flea3::Run()
{
	if(!is_init){return Mat();}

	std::lock_guard<std::mutex> lock(_mutex);
	thread_mat.copyTo(capture_mat);

	return capture_mat;
}

#if 0

// simple test

int main()
{
	Flea3 flea3(0);

	flea3.SetShutter(0.061f);
	flea3.SetExposure(2.4f);
	flea3.SetGamma(4.0f);
	flea3.SetGain(18.1f);

	while (true)
	{
		int input_key = waitKey(10);
		
		if(input_key == 'q'){ break;	}
		
		Mat capture_image = Flea3.Run();
		imshow("Test window", capture_image);
	}

	return 0;
}

#endif 