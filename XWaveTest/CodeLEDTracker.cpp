#include "CodeLEDTracker.h"

void CodeLEDTracker::Initialize(Mat in_image)
{
	tracking_threshold = 30.0f;
	code_led.Initialize(in_image);
}

void CodeLEDTracker::Run(Mat in_image, std::vector<PointData>& out_points)
{
	std::vector<PointData> code_points;
	std::vector<PointData> next_tracking_points;
	code_led.Run(in_image, code_points);

//	printf("Candidates %zd\n", code_points.size());

	for (int i = 0; i < code_points.size(); i++)
	{
		// with id
		if(code_points[i].id != -1)
		{ 
			int j;
			// find matching with existing point
			for (j = 0; j < tracking_points.size(); j++)
			{
				if (code_points[i].id == tracking_points[j].id) {
					code_points[i].duration = tracking_points[j].duration + 1;	// duration increment
					next_tracking_points.push_back(code_points[i]);
					break;
				}
			}

			// if not found, add as new point
			if (j == tracking_points.size()) {
				next_tracking_points.push_back(code_points[i]);
			}
		}
		// without id
		else
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
				code_points[i].duration = tracking_points[min_id].duration-1;						// decrement score
				if(code_points[i].duration > 0){
					next_tracking_points.push_back(code_points[i]);
				}
			}
		}
	}

	tracking_points = next_tracking_points;
	out_points = tracking_points;
}

void CodeLEDTracker::Finalize()
{
	code_led.Finalize();
}


#ifdef __CODE_LED_TRACKER_TEST__

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

	CodeLEDTracker x_tracker;

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
			cv::rectangle(input_mat, cv::Point(point_data[i].position.x - 20.0f, point_data[i].position.y - 20.0f), cv::Point(point_data[i].position.x + 20.0f, point_data[i].position.y + 20.0f), cv::Scalar(200, 200, 200), 5, 8);
			printf("id:%d duration:%d  x:%f, y:%f\n", point_data[i].id, point_data[i].duration, point_data[i].position.x, point_data[i].position.y);
		}
		
		if(point_data.size() == 0) {
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

#endif // __CODE_LED_TRACKER_TEST__


