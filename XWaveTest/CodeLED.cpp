#include <stdio.h>
#include "CodeLED.h"

char debug_log[100000];


void CodeLED::Initialize(Mat in_image)
{
	// initialize setting values
	window_width = 40;
	window_height = 400;
	brightness_threshold = 150;
	min_paint_threshold = 10;
	max_paint_threshold = 1000;
	peak_threshold = 25.0f;

	code_threshold[0] = 8;
	code_threshold[1] = 15;

	decode_length_threshold = 12;

	// for cog detection
	element_erode = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(3, 3));
	element_dilate = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(11, 11));

	labels_image = Mat(in_image.size(), CV_16U);

	// for peak detection
	sum_image = Mat(Size(window_width, window_height), CV_32F);
	sobel_image = cv::Mat(Size(window_width, window_height), CV_32F);
	barcode_image = cv::Mat(Size(window_width, window_height), CV_8U);
}

int CodeLED::DynamicThresholding()
{
	int max_value = 0;
	for (int i = 0; i < gray_image.size().width * gray_image.size().height; i++) {
		if (max_value < gray_image.data[i]) { max_value = gray_image.data[i]; }
	}

	if(max_value - 80 < 10)
		return 10;
	else 
		return max_value - 80;
}

void CodeLED::Run(Mat in_image, std::vector<PointData>& out_points)
{
	cvtColor(in_image, gray_image, CV_BGRA2GRAY);	// Should be only 8bit or more...?

	brightness_threshold = DynamicThresholding();
//	printf("Threshold: %d\n", brightness_threshold);

	DetectCenter(brightness_threshold, min_paint_threshold, max_paint_threshold, out_points);

	for (int i = 0; i < out_points.size(); i++)
	{
		SetROI(window_width, window_height, out_points[i].position);
		GenerateBarcode();
		DecodeID(out_points[i]);
//		printf("id:%d x:%f, y:%f\n", out_points[i].id, out_points[i].position.x, out_points[i].position.y);
//		cv::imshow("ROI", roi_image);
//		cv::imshow("Barcode", barcode_image);
	}
}

void CodeLED::DecodeID(PointData& inout_point)
{
	std::vector<int>	       diff_width;
	std::vector<unsigned char> decode_array;
	bool is_up = false;
	int  diff_pos = window_height - 1;

	// signal length measurement
	for (int y = 0; y < window_height; y++)
	{
		if (barcode_image.at<unsigned char>(y, 0) > 0)
		{
			if (!is_up)
			{
				diff_width.push_back(-(y - diff_pos));
				diff_pos = y;
			}

			is_up = true;
		}
		else
		{
			if (is_up)
			{
				diff_width.push_back((y - diff_pos));
				diff_pos = y;
			}

			is_up = false;
		}
	}

	// decode signal blank to binary
	for (int i = 0; i<diff_width.size(); i++)
	{
		if (code_threshold[0] >= abs(diff_width[i])) { decode_array.push_back(0); }
		else if (code_threshold[0] < abs(diff_width[i]) && code_threshold[1] >= abs(diff_width[i])) { decode_array.push_back(1); }
		else if (code_threshold[1] < abs(diff_width[i])) { decode_array.push_back(2); }	// used as header code
	}

	// decode binary to ID
	bool is_head = false;
	int digit = 0;
	unsigned char decode_id = 0;
	unsigned char decode_parity = 0;

	if(decode_array.size() < decode_length_threshold) return;

	// left/right bi-direction scanning
	for (int i = 0; i < decode_array.size(); i++)
	{
		if (decode_array[i] == 2)
		{
			int min_offset = 1;

			if (i < 12)
			{
				min_offset = 12-i;
//				printf("Initial offset:%d\n", min_offset);
			}

			for(;min_offset <= 12 && i + min_offset < decode_array.size(); min_offset++)
			{
				decode_id = 0;
				decode_parity = 0;

				for(int digit = 1; digit <= 11; digit++)
				{
					int j;

					if (digit >= min_offset) {
						j = i + digit - 12;
					}
					else {
						j = i + digit;
					}

					if(decode_array[j] == 2)
					{
						decode_id = 0;
						break; 
					}

					// data 8 bit
					if (digit > 0 && digit <= 8) {	
 						decode_id = decode_id | (decode_array[j] << (8 - digit));
					}
					// parity 3 bit
					else 
					{			
						decode_parity = decode_parity | (decode_array[j] << (11 - digit));

						// full bit check
						if (digit == 11)
						{
							int parity = ((int)((decode_id >> 7) & 0b00000001)
										+ (int)((decode_id >> 6) & 0b00000001)
										+ (int)((decode_id >> 5) & 0b00000001)
										+ (int)((decode_id >> 4) & 0b00000001)
										+ (int)((decode_id >> 3) & 0b00000001)
										+ (int)((decode_id >> 2) & 0b00000001)
										+ (int)((decode_id >> 1) & 0b00000001)
										+ (int)((decode_id) & 0b00000001))
										& 0b00000111;

							if (decode_parity == parity) 
							{
								inout_point.id = (int)decode_id; 
//								printf("COMPLETE: offset:%d\n", min_offset);
							}
							else {
//								printf("PARITY ERROR: code:%d  parity:%d\n", decode_id, decode_parity);
								decode_id = 0;
							}
						}
					}
				}

				if(decode_id > 0){ break; }
			}
		}

		if(decode_id > 0){ break; }
	}

/*	// 
	// left-to-right scanning
	for (int i = 0; i < decode_array.size(); i++)
	{
		if (decode_array[i] == 2) 
		{
			digit = 0;
			is_head = true; 
			decode_id = 0;
			decode_parity = 0;
		}

		if (digit > 0 && digit <= 8 && is_head) {						// data 8 bit
			decode_id = decode_id | (decode_array[i] << (8 - digit));
		}
		else if (digit > 8 && digit <= 11 && is_head) {					// parity 3 bit
			decode_parity = decode_parity | (decode_array[i] << (11 - digit));

			if (digit == 11)
			{
				int parity = ((int)((decode_id >> 7) & 0b00000001)
							+ (int)((decode_id >> 6) & 0b00000001)
							+ (int)((decode_id >> 5) & 0b00000001)
							+ (int)((decode_id >> 4) & 0b00000001)
							+ (int)((decode_id >> 3) & 0b00000001)
							+ (int)((decode_id >> 2) & 0b00000001)
							+ (int)((decode_id >> 1) & 0b00000001)
							+ (int)((decode_id) & 0b00000001))
							& 0b00000111;
				if (decode_parity == parity) { inout_point.id = (int)decode_id; break; }
				else{
					printf("PARITY ERROR: code:%d  parity:%d\n", decode_id, decode_parity);
				}
				is_head = false;
			}
		}

		digit++;
	}
*/
//	printf("array length:%d decide id:%d\n", decode_array.size(), decode_id);

	if (decode_array.size() > 15 && decode_id == 0)
//	if(false)
	{
		FILE* fp = fopen("Debug.csv", "w");
		if(fp)
		{
			fprintf(fp, "%s", debug_log);
			fclose(fp);
		}

		FILE* fp2 = fopen("Debug2.csv", "w");
		if (fp2)
		{
			fprintf(fp2, ", %d", diff_width[0]);

			for (int i = 1; i < diff_width.size(); i++) {
				fprintf(fp2, ", %d", diff_width[i]);
			}
			fprintf(fp2, "\n");

			fprintf(fp2, ", %d", decode_array[0]);

			for (int i = 1; i < decode_array.size(); i++) {
				fprintf(fp2, ", %d", decode_array[i]);
			}
			fprintf(fp2, "\n");

			fclose(fp2);
		}
	}
}

void CodeLED::GenerateBarcode()
{
	float ave_value = 0.0f;

	// summing at each y
	for (int y = 0; y < window_height; y++)
	{
		float sum_value = 0.0f;
		int sum_count = 0;

		for (int x = 0; x < window_width; x++)
		{
			unsigned short label_value = roi_labels_image.at<unsigned short>(y, x);
			if (label_value == 0)
			{
				sum_value += (float)(roi_image.at<unsigned char>(y, x));
				sum_count++;
			}
		}

		// sum adjustment
//		if (sum_count > 0 && sum_count < window_width) {
//			sum_value = sum_value * (float)window_width / sum_count;
//		}

		ave_value += sum_value;

		for (int x = 0; x < window_width; x++) {
			sum_image.at<float>(y, x) = sum_value;
		}
	}

	ave_value /= window_width;

//	cv::imshow("SUM", sum_image / 10000.0f);

	cv::GaussianBlur(sum_image, sum_image, cv::Size(1, 5), 0.0f);
	cv::Sobel(sum_image, sobel_image, CV_32F, 0, 1, 5);

	barcode_image.setTo(0);

	float peak_value = ave_value;
	float bottom_value = ave_value;
	bool  is_peak = false;
	bool  is_bottom = false;
	bool  is_up = false;

	float min_threshold_array[1048];
	float max_threshold_array[1048];

	for (int y = 0; y < window_height - 1; y++)
	{
		float sobel_value1 = sobel_image.at<float>(y, 0);
		float sobel_value2 = sobel_image.at<float>(y + 1, 0);

		if (sobel_value1 > 0
		 && sobel_value2 <= 0) {
			float peak_rate = 1.0f - sobel_value1 / (sobel_value1 - sobel_value2);
			float new_peak = sum_image.at<float>(y, 0) * peak_rate + sum_image.at<float>(y + 1, 0) * (1.0f - peak_rate);

			if (new_peak - bottom_value > peak_threshold)
			{
				peak_value = new_peak;
				is_peak = true;
				is_up = true;

				if (is_peak
					&& is_bottom)
				{
					float middle_threshold = (peak_value + bottom_value) / 3.0f;		// ideally, 2.0. But actually it's lower.
					int current_y = y - 1;

					while (current_y >= 0)
					{
						if(middle_threshold > sum_image.at<float>(current_y, 0)){	break;	}
						barcode_image.at<unsigned char>(current_y, 0) = 255;
						current_y--;
					}
					// sub pixel computation
					/*
					float cross_value1 = middle_threshold - sum_image.at<float>(0, current_x);
					float cross_value2 = sum_image.at<float>(0, current_x + 1) - middle_threshold;
					float cross_rate = 1.0f - cross_value1 / (cross_value1 + cross_value2);
					if (cross_rate < 0.5f){ barcode_image.at<unsigned char>(0, current_x) = (unsigned char)(int)(255 * cross_rate * 2.0f);  }
					else { barcode_image.at<unsigned char>(0, current_x + 1) = (unsigned char)(int)(255 * (1.0f - cross_rate) * 2.0f); }
					*/
				}
			}
		}
		else if (sobel_value1 <= 0
			&& sobel_value2 > 0)
		{
			float peak_rate = 1.0f + sobel_value1 / (-sobel_value1 + sobel_value2);
			float new_bottom = sum_image.at<float>(y, 0) * peak_rate + sum_image.at<float>(y + 1, 0) * (1.0f - peak_rate);

			if (peak_value - new_bottom > peak_threshold)
			{
				bottom_value = new_bottom;
				is_bottom = true;
				is_up = false;

				if (is_peak
					&& is_bottom)
				{
					float middle_threshold = (peak_value + bottom_value) / 3.0f;		// ideally, 2.0. But actually it's lower.
					int current_y = y - 1;
					while (current_y >= 0)
					{
						if(middle_threshold < sum_image.at<float>(current_y, 0)){	break;	}
						barcode_image.at<unsigned char>(current_y, 0) = 0;
						current_y--;
					}
					// sub pixel computation
					/*
					float cross_value1 = sum_image.at<float>(0, current_x) - middle_threshold;
					float cross_value2 = middle_threshold - sum_image.at<float>(0, current_x + 1);
					float cross_rate = 1.0f - cross_value1 / (cross_value1 + cross_value2);
					if (cross_rate < 0.5f) { barcode_image.at<unsigned char>(0, current_x) = (unsigned char)(int)(255 * cross_rate * 2.0f); }
					else { barcode_image.at<unsigned char>(0, current_x + 1) = (unsigned char)(int)(255 * (1.0f - cross_rate) * 2.0f); }
					*/
				}
			}
		}

		max_threshold_array[y] = peak_value;
		min_threshold_array[y] = bottom_value;

		if (is_up)
		{
			barcode_image.at<unsigned char>(y, 0) = 255;
		}
		else
		{
			barcode_image.at<unsigned char>(y, 0) = 0;
		}
	}

	// for debug log
//	FILE* fp = fopen("Debug.csv", "w");
//	if (fp)
	{
		sprintf(debug_log, ", %f", sum_image.at<float>(0, 0));

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %f", debug_log, sum_image.at<float>(y, 0));
		}
		sprintf(debug_log, "%s\n", debug_log);

		sprintf(debug_log, "%s, %d", debug_log, barcode_image.at<unsigned char>(0, 0));

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %d", debug_log, barcode_image.at<unsigned char>(y, 0));
		}
		sprintf(debug_log, "%s\n", debug_log);

		sprintf(debug_log, "%s, %f", debug_log, max_threshold_array[0]);

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %f", debug_log, max_threshold_array[y]);
		}
		sprintf(debug_log, "%s\n", debug_log);

		sprintf(debug_log, "%s, %f", debug_log, min_threshold_array[0]);

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %f", debug_log, min_threshold_array[y]);
		}
		sprintf(debug_log, "%s\n", debug_log);

//		fclose(fp);
	}

	// genrate barcode
	for (int y = 0; y < window_height; y++) 
	{
		for (int x = 0; x < window_width; x++)
		{
			barcode_image.at<unsigned char>(y, x) = barcode_image.at<unsigned char>(y, 0);
		}
	}
}

void CodeLED::SetROI(int in_width, int in_height, Point2f in_position)
{
	int center_x = (int)in_position.x;
	int center_y = (int)in_position.y;

	int leftup_x = center_x - in_width / 2;
	int leftup_y = center_y - in_height / 2;

	if (leftup_x < 0) { leftup_x = 0; }
	if (leftup_y < 0) { leftup_y = 0; }

	if (leftup_x + in_width > gray_image.size().width - 1) { leftup_x = gray_image.size().width - in_width - 1; }
	if (leftup_y + in_height > gray_image.size().height - 1) { leftup_y = gray_image.size().height - in_height - 1; }

	roi_image = Mat(gray_image, Rect(leftup_x, leftup_y, in_width, in_height));
	roi_labels_image = Mat(labels_image, Rect(leftup_x, leftup_y, in_width, in_height));
}

void CodeLED::DetectCenter(int in_peak_threshold, int in_min_area_threshold, int in_max_area_threshold, std::vector<PointData>& inout_points)
{
	GaussianBlur(gray_image, blur_image, cv::Size(3, 3), 0);
	threshold(blur_image, thresh_image, in_peak_threshold, 255, CV_THRESH_BINARY);
	erode(thresh_image, thresh_image, element_erode);
	dilate(thresh_image, thresh_image, element_dilate);

	int num_labels = cv::connectedComponents(thresh_image, labels_image, 4, CV_16U);
	std::vector<int> label_counter;
	label_counter.resize(num_labels);

	std::vector<Point2f> label_points;
	label_points.resize(num_labels);

	int index = 0;

	for (int y = 0; y < labels_image.size().height; y++)
	{
		for (int x = 0; x < labels_image.size().width; x++)
		{
			unsigned short label_data = ((unsigned short*)labels_image.data)[index++];
			if (label_data > 0)
			{
				label_counter[label_data - 1]++;
				label_points[label_data - 1].x += (float)x;
				label_points[label_data - 1].y += (float)y;
			}
		}
	}

	inout_points.clear();
	for (int j = 0; j < num_labels; j++)
	{
		if (label_counter[j] > in_min_area_threshold
			&&  label_counter[j] < in_max_area_threshold)
		{
			PointData new_point_data;
			new_point_data.position = Point2f(label_points[j].x / label_counter[j], label_points[j].y / label_counter[j]);
			inout_points.push_back(new_point_data);
//			printf("label:%d num:%d x:%f, y:%f\n", j, label_counter[j], new_point_data.position.x, new_point_data.position.y);
		}
	}

	// for debug
	mask_image = cv::Mat(labels_image.size(), CV_8U);
	mask_image.setTo(0);

	for (int i = 0; i < labels_image.size().width * labels_image.size().height; i++)
	{
		unsigned short label_data = ((unsigned short*)labels_image.data)[i];
		if (label_data > 0)
		{
			if (label_counter[label_data - 1] < in_min_area_threshold || label_counter[label_data - 1] > in_max_area_threshold)
			{
				((unsigned short*)labels_image.data)[i] = 0;
			}
			// debug code
			else 
			{
				mask_image.data[i] = 255;
			}
		}
	}

	cv::imshow("Labeled", mask_image);
}

#ifdef __CODE_LED_TEST__

// Unit test
int main()
{
	bool video_mode = false;
	VideoCapture camera_capture;

	if(video_mode == true)
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
	cv::Mat save_mat;

	if(video_mode)
		camera_capture >> input_mat;
	else
		input_mat = cv::imread("rawimage.png");

	input_mat.copyTo(save_mat);

	for(int i=0; i<10; i++)	input_mat.copyTo(visual_mat[i]);
	output_mat = cv::Mat(input_mat.size(), input_mat.type());

	CodeLED x_detector;

	x_detector.Initialize(input_mat);
	std::vector<PointData> point_data;

	int current_index = 0;

	while (true)
	{
		if(video_mode)
			camera_capture >> input_mat;

		int input_key = cv::waitKey(1);
		if (input_key == ' ') break;

		x_detector.Run(input_mat, point_data);
		cv::imshow("input", input_mat);

		for (int i = 0; i < point_data.size(); i++)
		{
			printf("Point X:%.2f Y:%.2f DURATION:%d ID:%d\n", point_data[i].position.x, point_data[i].position.y, point_data[i].duration, point_data[i].id);
			if(point_data[i].id == -1){ input_mat.copyTo(save_mat); }
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

	if(video_mode)
		cv::imwrite("rawimage.png", save_mat);

	return 0;
}

#endif // __CODE_LED_TEST__


