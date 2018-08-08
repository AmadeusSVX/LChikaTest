#include "CodeLEDComm.h"

void CodeLEDComm::Initialize(Mat in_image)
{
	led_comm_interval = 1000.0f;
	led_comm_bitrate = 100.0f;

	tracking_threshold = 400.0f;
	code_led.Initialize(in_image);
}

void CodeLEDComm::Run(Mat in_image, std::vector<PointData>& out_points)
{
	std::vector<PointData> code_points;
	std::vector<PointData> current_tracking_points;
	std::vector<PointData> next_tracking_points;
	code_led.Run(in_image, code_points);

//	printf("Candidates %zd\n", code_points.size());

	for (int i = 0; i < code_points.size(); i++)
	{
		float min_distance = tracking_threshold;
		int min_id = -1;

		for (int j = 0; j < tracking_points.size(); j++)
		{
			float distance = (float)norm(code_points[i].position - tracking_points[j].position);

			if (distance < min_distance)
			{
				min_id = j;
				min_distance = distance;
			}
		}

		if (min_id != -1) {		
			//	found
			// copy data to next
			code_points[i].duration = tracking_points[min_id].duration + 1;
			code_points[i].data_status = tracking_points[min_id].data_status;
			code_points[i].temp_data = tracking_points[min_id].temp_data;
			code_points[i].data = tracking_points[min_id].data;

			current_tracking_points.push_back(tracking_points[min_id]);
			next_tracking_points.push_back(code_points[i]);
		}
		else
		{
			// if not found, add as new point
			current_tracking_points.push_back(code_points[i]);
			next_tracking_points.push_back(code_points[i]);
		}
	}

	for (int i = 0; i < next_tracking_points.size(); i++)
	{
//		printf("NO: %d ID0: %d ID1: %d x:%f y:%f duration:%d\n", i, current_tracking_points[i].id & 0b10000000, next_tracking_points[i].id & 0b10000000, next_tracking_points[i].position.x, next_tracking_points[i].position.y, next_tracking_points[i].duration);
//		 check header change
		if ((current_tracking_points[i].id & 0b10000000) != (next_tracking_points[i].id & 0b10000000))
		{
//			printf("Bit change\n");
			unsigned char data_byte = next_tracking_points[i].id & 0b01111111;
			if(data_byte == 2)		// header
			{
//				printf("Data start\n");
				if(next_tracking_points[i].data_status == INPUT_END)
				{
					next_tracking_points[i].data_status = UPDATING;
				}
				else
				{
					next_tracking_points[i].data_status = INPUTTING;
				}
				tick_meter.reset();
				tick_meter.start();
				next_tracking_points[i].temp_data.clear(); 
			}
			else if(data_byte == 3 && (next_tracking_points[i].data_status == INPUTTING || next_tracking_points[i].data_status == UPDATING))	// footer
			{
//				printf("Data end\n");
				next_tracking_points[i].data_status = INPUT_END;
				next_tracking_points[i].data = next_tracking_points[i].temp_data;
				tick_meter.stop();
				led_comm_interval = (float)tick_meter.getTimeSec();
				led_comm_bitrate = next_tracking_points[i].data.length() * 8.0f / led_comm_interval;
			}
			else if(next_tracking_points[i].data_status == INPUTTING || next_tracking_points[i].data_status == UPDATING)
			{
//				printf("Data input\n");
				next_tracking_points[i].temp_data.push_back(data_byte);
			}
		}
	}

	tracking_points = next_tracking_points;
	out_points = tracking_points;
}

void CodeLEDComm::DecodeData()
{

}

void CodeLEDComm::Finalize()
{
	code_led.Finalize();
}


#ifdef __CODE_LED_COMM_TEST__

#include "Flea3.h"

// Unit test
int main()
{
	int video_mode = 0;
	bool video_record = true;
	VideoCapture video_reader;
	Flea3* camera_capture = NULL;

	VideoWriter video_writer;

	Mat color_mat;

	if (video_mode == 0)
	{
		camera_capture = new Flea3(0);
		camera_capture->SetShutter(0.031f);
		camera_capture->SetExposure(2.4f);
		camera_capture->SetGamma(4.0f);
		camera_capture->SetGain(18.1f);

		if(video_record)
			video_writer = VideoWriter(".\\CommVideo.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, Size(1280, 960));

		//		camera_capture.open(0);
		//		camera_capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
		//		camera_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 960);
		//		camera_capture.set(CV_CAP_PROP_EXPOSURE, -12.0f);
		//		camera_capture.set(CV_CAP_PROP_GAIN, 300.0f);
	}
	else if (video_mode == 1)
	{
		video_reader = VideoCapture(".\\OutVideo.avi");
	}
	else
		color_mat = imread("debugimage.png");


	cv::Mat input_mat;
	cv::Mat output_mat;
	cv::Mat save_mat;


	if (video_mode == 0)
		//		camera_capture >> input_mat;
		input_mat = camera_capture->Run();
	else if (video_mode == 1)
	{
		video_reader >> color_mat; // = cv::imread("rawimage1.png");
		cvtColor(color_mat, input_mat, CV_BGR2GRAY);
	}
	else
	{
		cvtColor(color_mat, input_mat, CV_BGR2GRAY);
	}

	output_mat = cv::Mat(input_mat.size(), input_mat.type());

	CodeLEDComm x_tracker;

	x_tracker.Initialize(input_mat);
	std::vector<PointData> point_data;

	int current_index = 0;

	while (true)
	{
		if (video_mode == 0)
			input_mat = camera_capture->Run();
		else if (video_mode == 1)
		{
			video_reader >> color_mat;
			if (color_mat.cols == 0) break;
			cvtColor(color_mat, input_mat, CV_BGR2GRAY);
		}
		else
			cvtColor(color_mat, input_mat, CV_BGR2GRAY);

		cvtColor(input_mat, output_mat, CV_GRAY2BGR);

		int input_key = cv::waitKey(1);
		if (input_key == ' ') break;

		x_tracker.Run(input_mat, point_data);

		for (int i = 0; i < point_data.size(); i++) {
			if(point_data[i].data_status == INPUT_END || point_data[i].data_status == UPDATING)
			{ 
				cv::rectangle(output_mat, cv::Point(point_data[i].position.x - 20.0f, point_data[i].position.y - 20.0f), cv::Point(point_data[i].position.x + 20.0f, point_data[i].position.y + 20.0f), cv::Scalar(0, 200, 0), 5, 8);
				cv::putText(output_mat, point_data[i].data.c_str(), cv::Point(point_data[i].position.x, point_data[i].position.y - 50.0f), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 200, 0), 2, CV_AA);
				printf("id:%d duration:%d speed:%.2f bps --%s-- x:%f, y:%f\n", point_data[i].id, point_data[i].duration, x_tracker.GetBitRate(), point_data[i].data.c_str(), point_data[i].position.x, point_data[i].position.y);
			}
		}

		cv::imshow("input", input_mat);

		if (point_data.size() == 0) {
			printf("nothing detected\n");
		}

		current_index = (current_index + 1) % 10;
		cv::imshow("out", output_mat);

		if (video_mode == 0 && video_record)
		{
			cvtColor(input_mat, color_mat, CV_GRAY2BGR);
			video_writer << output_mat;
		}

	}

	//	if(video_mode)
	//		cv::imwrite("rawimage.png", input_mat);

	waitKey(0);

	return 0;
}

#endif // __CODE_LED_COMM_TEST__


