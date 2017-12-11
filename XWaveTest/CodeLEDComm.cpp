#include "CodeLEDComm.h"

void CodeLEDComm::Initialize(Mat in_image)
{
	tracking_threshold = 30.0f;
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
				next_tracking_points[i].data_status = INPUTTING;
				next_tracking_points[i].data.clear(); 
			}
			else if(data_byte == 3 && next_tracking_points[i].data_status == INPUTTING)	// footer
			{
//				printf("Data end\n");
				next_tracking_points[i].data_status = INPUT_END;
			}
			else if(next_tracking_points[i].data_status == INPUTTING)
			{
//				printf("Data input\n");
				next_tracking_points[i].data.push_back(data_byte);
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

// Unit test
int main()
{
	bool video_mode = true;
	VideoCapture camera_capture;

	if (video_mode == true)
	{
		camera_capture.open(2);
		camera_capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
		camera_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
		camera_capture.set(CV_CAP_PROP_EXPOSURE, -12.0f);
		camera_capture.set(CV_CAP_PROP_GAIN, 300.0f);
	}

	cv::Mat input_mat;
	cv::Mat visual_mat[10];
	cv::Mat output_mat;

	if (video_mode)
		camera_capture >> input_mat;
	else
		input_mat = cv::imread("rawimage.png");

	for (int i = 0; i<10; i++)	input_mat.copyTo(visual_mat[i]);
	output_mat = cv::Mat(input_mat.size(), input_mat.type());

	CodeLEDComm x_tracker;

	x_tracker.Initialize(input_mat);
	std::vector<PointData> point_data;

	int current_index = 0;

	while (true)
	{
		if (video_mode)
			camera_capture >> input_mat;

		int input_key = cv::waitKey(1);
		if (input_key == ' ') break;

		x_tracker.Run(input_mat, point_data);
		cv::imshow("input", input_mat);

		for (int i = 0; i < point_data.size(); i++) {
			if(point_data[i].data_status == INPUT_END)
				printf("id:%d duration:%d --%s-- x:%f, y:%f\n", point_data[i].id, point_data[i].duration, point_data[i].data.c_str(), point_data[i].position.x, point_data[i].position.y);
		}

		if (point_data.size() == 0) {
			printf("nothing detected\n");
		}

		output_mat.setTo(cv::Scalar(0, 0, 0));
		input_mat.copyTo(visual_mat[current_index]);
		for (int i = 0; i < 10; i++)
		{
			output_mat = output_mat + visual_mat[i];
		}
		current_index = (current_index + 1) % 10;

		cv::imshow("accum out", output_mat);

	}

	//	if(video_mode)
	//		cv::imwrite("rawimage.png", input_mat);

	return 0;
}

#endif // __CODE_LED_COMM_TEST__


