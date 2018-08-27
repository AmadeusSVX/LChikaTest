#include <stdio.h>
#include "CodeLED.h"

#include <iostream>
#include <omp.h>

char debug_log[100000];

void CodeLED::Initialize(Mat in_image)
{
	// initialize setting values
	window_width = 20;
	window_height = 800;
	brightness_threshold = 50;
	min_paint_threshold = 10;
	max_paint_threshold = 1000;
	min_diff_threshold = 30.0f;
//	peak_threshold = 40.0f;
	peak_threshold = 20.0f;

	code_threshold[0] = 8;
	code_threshold[1] = 18;

	code_interval = 177;		// for Arduino Uno
//	code_interval = 188;		// for Studino (Overhead is happening?)

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

	// 0.6ms
//	cvtColor(in_image, gray_image, CV_BGRA2GRAY);	// Should be only 8bit or more...?
	in_image.copyTo(gray_image);

	// 1.5ms
//	brightness_threshold = DynamicThresholding();
//	printf("Threshold: %d\n", brightness_threshold);

	DetectCenter(brightness_threshold, min_paint_threshold, max_paint_threshold, out_points);	// 9ms
//	DetectCenter2(brightness_threshold, min_paint_threshold, max_paint_threshold, out_points);	// 12ms

	for (int i = 0; i < out_points.size(); i++)
	{
		SetROI(window_width, window_height, out_points[i].position);
		GenerateBarcode();
		DecodeID2(out_points[i]);

//		printf("id:%d x:%f, y:%f\n", out_points[i].id, out_points[i].position.x, out_points[i].position.y);
//		cv::imshow("ROI", roi_image);
//		cv::imshow("Barcode", barcode_image);
	}
}

void CodeLED::DecodeID2(PointData& inout_point)
{
	std::vector<int>			diff_width;
	std::vector<unsigned char>	decode_array;
	std::vector<int>			head_start;
	std::vector<int>			head_end;

	bool is_up = false;
	bool is_first = true;

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
				int new_diff_width = y - diff_pos;
				// header detection
				if(new_diff_width > code_threshold[1])
				{
					head_start.push_back(diff_pos);
					head_end.push_back(y);
				}
				diff_width.push_back((y - diff_pos));
				diff_pos = y;
			}

			is_up = false;
		}
	}

	// preparing for decode
	int new_code_interval = code_interval;

	if(head_start.size() >= 2)
	{
		int estimate_code_interval = head_start[1] - head_end[0];
//		printf("new interval %d\n", estimate_code_interval);

		if(abs(code_interval - (head_start[1] - head_end[0])) < 5)
		{
			new_code_interval = head_start[1] - head_end[0];
//			printf("new interval %d\n", new_code_interval);
		}
	}

	unsigned char code_filled = 0; // 0:not decoded 1:decoded for each bit
	unsigned char parity_filled = 0; // 0:not decoded 1:decoded for each bit
	float code_strength[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};	// strength computed by diff of brightness

	unsigned char decode_id = 0;
	unsigned char prev_decode_id = 0;
	unsigned char decode_parity = 0;

	std::vector<int>	y1_pos;
	std::vector<int>	y2_pos;

	// forward search
	for (int i = 0; i < head_end.size(); i++)
	{
		// code interval update
		for (int j = 0; j < head_start.size(); j++)
		{
			if (abs(code_interval - (head_start[j] - head_end[i])) < 10)
			{
				new_code_interval = head_start[j] - head_end[i];
			}
		}

		float unit_code = new_code_interval / 24.0f;

		// code input
		for (int j = 0; j < 11; j++)
		{
			int target_y1 = head_end[i] + int((1.5 + j * 2.0f) * unit_code - 0.5f);
			int target_y2 = head_end[i] + int((2.5 + j * 2.0f) * unit_code - 0.5f);

			// window boundary check
			if(target_y1 >= window_height - 1 || target_y2 >= window_height - 1){	break;	}

			y1_pos.push_back(target_y1);
			y2_pos.push_back(target_y2);

			float diff_value = sum_image.at<float>(target_y2, 0) - sum_image.at<float>(target_y1, 0);

			if(j < 8)
			{ 
				// code input
				if (diff_value > 0 && fabs(diff_value) > min_diff_threshold && code_strength[j] < fabs(diff_value))		// as 1
				{
					decode_id = decode_id | 0b1 << (7 - j);
					code_filled = code_filled | 0b1 << j;
					code_strength[j] = fabs(diff_value);
				}
				else if (fabs(diff_value) > min_diff_threshold && code_strength[j] < fabs(diff_value))					// as 0
				{
					decode_id = decode_id & ~(0b1 << (7 - j));
					code_filled = code_filled | 0b1 << j;
					code_strength[j] = fabs(diff_value);
				}
			}
			else
			{
				// parity input
				if (diff_value > 0 && fabs(diff_value) > min_diff_threshold && code_strength[j] < fabs(diff_value))		// as 1
				{
					decode_parity = decode_parity | 0b1 << (10 - j);
					parity_filled = parity_filled | 0b1 << (j - 8);
					code_strength[j] = fabs(diff_value);
				}
				else if (fabs(diff_value) > min_diff_threshold && code_strength[j] < fabs(diff_value))					// as 0
				{
					decode_parity = decode_parity & ~(0b1 << (10 - j));
					parity_filled = parity_filled | 0b1 << (j - 8);
					code_strength[j] = fabs(diff_value);
				}
			}
		}

		// if code filled
		if (code_filled == 255 && parity_filled == 0b111)
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
//				printf("COMPLETE with Forward search\n");
				return;
			}
			else if (prev_decode_id == decode_id && decode_id != 0)
			{
				inout_point.id = (int)decode_id;
//				printf("COMPLETE with Forward search\n");
				return;
			}
			else 
			{
				prev_decode_id = decode_id;
				decode_id = 0;
				// code reset
				code_filled = 0;
				parity_filled = 0;
	//			printf("PARITY ERROR: code:%d  parity:%d\n", decode_id, decode_parity);
			}
		}
	}

	code_filled = 0; // 0:not decoded 1:decoded for each bit
	parity_filled = 0; // 0:not decoded 1:decoded for each bit
	for(int i=0; i<11; i++) {code_strength[i] = 0.0f;	};	// strength computed by diff of brightness
	decode_id = 0;
	decode_parity = 0;

	// backward search
	for (int i = 0; i < head_start.size(); i++)
	{
		// code interval update
		for (int j = 0; j < head_end.size(); j++)
		{
			if (abs(code_interval - (head_start[i] - head_end[j])) < 10)
			{
				new_code_interval = head_start[i] - head_end[j];
			}
		}

		float unit_code = new_code_interval / 24.0f;

		// code input
		for (int j = 0; j < 11; j++)
		{
			int target_y1 = head_start[i] - new_code_interval + int((1.5 + j * 2.0f) * unit_code - 0.5f);
			int target_y2 = head_start[i] - new_code_interval + int((2.5 + j * 2.0f) * unit_code - 0.5f);

			// window boundary check
			if (target_y1 < 0 || target_y2 < 0) { continue; }

			y1_pos.push_back(target_y1);
			y2_pos.push_back(target_y2);

			float diff_value = sum_image.at<float>(target_y2, 0) - sum_image.at<float>(target_y1, 0);

			if (j < 8)
			{
				// code input
				if (diff_value > 0 && fabs(diff_value) > 10 && code_strength[j] < fabs(diff_value))		// as 1
				{
					decode_id = decode_id | 0b1 << (7 - j);
					code_filled = code_filled | 0b1 << j;
					code_strength[j] = fabs(diff_value);
				}
				else if (fabs(diff_value) > 10 && code_strength[j] < fabs(diff_value))					// as 0
				{
					decode_id = decode_id & ~(0b1 << (7 - j));
					code_filled = code_filled | 0b1 << j;
					code_strength[j] = fabs(diff_value);
				}
			}
			else
			{
				// parity input
				if (diff_value > 0 && fabs(diff_value) > 10 && code_strength[j] < fabs(diff_value))		// as 1
				{
					decode_parity = decode_parity | 0b1 << (10 - j);
					parity_filled = parity_filled | 0b1 << (j - 8);
					code_strength[j] = fabs(diff_value);
				}
				else if (fabs(diff_value) > 10 && code_strength[j] < fabs(diff_value))					// as 0
				{
					decode_parity = decode_parity & ~(0b1 << (10 - j));
					parity_filled = parity_filled | 0b1 << (j - 8);
					code_strength[j] = fabs(diff_value);
				}
			}
		}

		// if code filled
		if (code_filled == 255 && parity_filled == 0b111)
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
//				printf("COMPLETE with Back search\n");
				return;
			}
			else if (prev_decode_id == decode_id && decode_id != 0)
			{
				inout_point.id = (int)decode_id;
//				printf("COMPLETE with Back search\n");
				return;
			}
			else
			{
				prev_decode_id = decode_id;
				decode_id = 0;
				// code reset
				code_filled = 0;
				parity_filled = 0;
				//			printf("PARITY ERROR: code:%d  parity:%d\n", decode_id, decode_parity);
			}
		}
	}
/*
	for (int i = 0; i < int(head_start.size()) - 1; i++)
	{
		printf("Head start:%d\n", head_start[i + 1]);
		printf("Head end:%d\n", head_end[i]);
		printf("Interval:%d\n", head_start[i + 1] - head_end[i]);
	}
*/

	return;

	if(head_start.size() > 1)
	{
		FILE* fp;

		if(inout_point.id == -1)
			fp = fopen("Debug.csv", "w");
		else
			fp = fopen("Debug0.csv", "w");

		if (fp)
		{
			fprintf(fp, "%s", debug_log);

			fprintf(fp, "0, ");

			for(int y=1; y<window_height; y++)
			{
				int i;
				for(i=0; i<y1_pos.size(); i++){ if(y1_pos[i] == y) break;	}
				if(i == y1_pos.size())	fprintf(fp, "0,");
				else fprintf(fp, "255,");
			}
			fprintf(fp, "\n");

			fprintf(fp, "0, ");

			for (int y = 1; y<window_height; y++)
			{
				int i;
				for (i = 0; i<y2_pos.size(); i++) { if (y2_pos[i] == y) break; }
				if (i == y2_pos.size())	fprintf(fp, "0,");
				else fprintf(fp, "255,");
			}
			fprintf(fp, "\n");
			fprintf(fp, "%d %d\n", (int)decode_id, (int)decode_parity);
			fclose(fp);
		}
	}

	return;

	// decode signal blank to binary
	for (int i = 0; i<diff_width.size(); i++)
	{
		if (code_threshold[0] >= abs(diff_width[i])) { decode_array.push_back(0); }
		else if (code_threshold[0] < abs(diff_width[i]) && code_threshold[1] >= abs(diff_width[i])) { decode_array.push_back(1); }
		else if (code_threshold[1] < abs(diff_width[i])) { decode_array.push_back(2); }	// used as header code
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
//			unsigned short label_value = roi_labels_image.at<unsigned short>(y, x);
//			if (label_value == 0)
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
//	cv::imshow("ROI", roi_image);
//	cv::imshow("SUM", sum_image / 3000.0f);
	cv::GaussianBlur(sum_image, sum_image, cv::Size(1, 7), 0.0f);
	cv::Sobel(sum_image, sobel_image, CV_32F, 0, 1, 5);
//	cv::imshow("SOBEL", sobel_image * 0.001f);

	barcode_image.setTo(0);

	float peak_value = ave_value;
	float bottom_value = ave_value;
	bool  is_peak = false;
	bool  is_bottom = false;
	bool  is_up = false;

	float min_threshold_array[1048];
	float max_threshold_array[1048];
	float vari_peak_threshold = peak_threshold;

	// generating barcode image
	for (int y = 0; y < window_height - 1; y++)
	{
		float sobel_value1 = sobel_image.at<float>(y, 0);
		float sobel_value2 = sobel_image.at<float>(y + 1, 0);

		// when passing high peak, here must be 1
		if (sobel_value1 > 0
		 && sobel_value2 <= 0) {
			float peak_rate = 1.0f - sobel_value1 / (sobel_value1 - sobel_value2);
			float new_peak = sum_image.at<float>(y, 0) * peak_rate + sum_image.at<float>(y + 1, 0) * (1.0f - peak_rate);

			// checking if small peak or not 
			if (new_peak - bottom_value > vari_peak_threshold)
			{
				peak_value = new_peak;
				is_peak = true;
				is_up = true;

				if (is_bottom)
				{
					float middle_threshold = (peak_value + bottom_value) / 2.0f;		// ideally, 2.0. But actually it's lower.
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
		// when passing low peak, it must be 0
		else if (sobel_value1 <= 0
		   	  && sobel_value2 > 0)
		{
			float peak_rate = 1.0f + sobel_value1 / (-sobel_value1 + sobel_value2);
			float new_bottom = sum_image.at<float>(y, 0) * peak_rate + sum_image.at<float>(y + 1, 0) * (1.0f - peak_rate);

			if (peak_value - new_bottom > vari_peak_threshold)
			{
				bottom_value = new_bottom;
				is_bottom = true;
				is_up = false;

				if (is_peak)
				{
					float middle_threshold = (peak_value + bottom_value) / 2.0f;		// ideally, 2.0. But actually it's lower.
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
		if((peak_value - bottom_value) * 0.005f > peak_threshold)
			vari_peak_threshold = (peak_value - bottom_value) * 0.005f;
		else
			peak_threshold = vari_peak_threshold;

		if (is_up)
		{
			barcode_image.at<unsigned char>(y, 0) = 255;
		}
		else
		{
			barcode_image.at<unsigned char>(y, 0) = 0;
		}
	}


//	return;
//	cv::imshow("barcode", barcode_image);

	// for debug log
	FILE* fp = fopen("Debug.csv", "w");
	if (fp)
	{
		sprintf(debug_log, "%f", sum_image.at<float>(0, 0));

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %f", debug_log, sum_image.at<float>(y, 0));
		}
		sprintf(debug_log, "%s\n", debug_log);

		sprintf(debug_log, "%s %d", debug_log, barcode_image.at<unsigned char>(0, 0));

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %d", debug_log, barcode_image.at<unsigned char>(y, 0));
		}
		sprintf(debug_log, "%s\n", debug_log);

		sprintf(debug_log, "%s %f", debug_log, max_threshold_array[0]);

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %f", debug_log, max_threshold_array[y]);
		}
		sprintf(debug_log, "%s\n", debug_log);

		sprintf(debug_log, "%s %f", debug_log, min_threshold_array[0]);

		for (int y = 1; y < window_height; y++) {
			sprintf(debug_log, "%s, %f", debug_log, min_threshold_array[y]);
		}
		sprintf(debug_log, "%s\n", debug_log);

		fprintf(fp, debug_log);
		fclose(fp);
	}

	return;
/*
	FILE* fp = fopen("Debug.csv", "w");
	if (fp)
	{
		fprintf(fp, debug_log);
		fclose(fp);
	}
*/
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
	threshold(gray_image, thresh_image, in_peak_threshold, 255, CV_THRESH_BINARY);

	erode(thresh_image, thresh_image, element_erode);
	dilate(thresh_image, thresh_image, element_dilate);


	int num_labels = cv::connectedComponents(thresh_image, labels_image, 8, CV_16U);
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

	std::vector<PointData> temp_points;
	for (int j = 0; j < num_labels; j++)
	{
		if (label_counter[j] > in_min_area_threshold
		&&  label_counter[j] < in_max_area_threshold)
		{
			PointData new_point_data;
			new_point_data.position = Point2f(label_points[j].x / label_counter[j], label_points[j].y / label_counter[j]);
			temp_points.push_back(new_point_data);
//			printf("label:%d num:%d x:%f, y:%f\n", j, label_counter[j], new_point_data.position.x, new_point_data.position.y);
		}
	}

	inout_points.clear();

//	inout_points = temp_points;
//	if(false)
	for (int j = 0; j < temp_points.size(); j++)
	{
		int k = 0;
		for (k = j; k < temp_points.size(); k++)
		{
			if (fabs(temp_points[j].position.x - temp_points[k].position.x) < 25)
			{

				if (blur_image.at<float>((int)temp_points[j].position.y, (int)temp_points[j].position.x) < blur_image.at<float>((int)temp_points[k].position.y, (int)temp_points[k].position.x))
				{
					break;
				}
			}
		}

		if (k == temp_points.size())
		{
			inout_points.push_back(temp_points[j]);
		}
	}

//	std::cout << inout_points.size() << std::endl;
/*
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
*/
}

void CodeLED::DetectCenter2(int in_peak_threshold, int in_min_area_threshold, int in_max_area_threshold, std::vector<PointData>& inout_points)
{
	horizontal_brightness = Mat(Size(gray_image.size().width, 1), CV_32F);
	horizontal_sobel = Mat(Size(gray_image.size().width, 1), CV_32F);	horizontal_brightness.setTo(0.0f);
	std::vector<PointData> point_candidates;

	vertical_sobel = Mat(Size(gray_image.size().width, gray_image.size().height), CV_32F);
	vertical_band_sobel = Mat(Size(20, gray_image.size().height), CV_32F);
/*
	tick_meter.reset();
	tick_meter.start();
	tick_meter.stop();
	static float tick_time = 0.0f;
	tick_time = tick_time * 0.9f + tick_meter.getTimeMilli() * 0.1f;
	printf("Process time: %f\n", tick_time);
*/
	Sobel(gray_image, vertical_sobel, CV_32F, 0, 1, 5);

	// generating integral image of vertical sobel image

	#pragma omp parallel for
	for (int x = 0; x < gray_image.size().width; x++)
	{
//		float* tgt_pointer = horizontal_brightness.ptr<float>(0);
		for (int y = 0; y < gray_image.size().height; y++)
		{
			horizontal_brightness.at<float>(0, x) += fabs(vertical_sobel.at<float>(y, x));
//			float* src_pointer = vertical_sobel.ptr<float>(y);
//			tgt_pointer[x] += fabs(src_pointer[x]);
		}
	}


	// smoothing
	GaussianBlur(horizontal_brightness, horizontal_brightness, Size(5, 1), 0);
	Sobel(horizontal_brightness, horizontal_sobel, CV_32F, 1, 0, 5);

	// x peak detection
	for (int x = 0; x < gray_image.size().width - 1; x++)
	{
		float first_value = horizontal_sobel.at<float>(0, x);
		float second_value = horizontal_sobel.at<float>(0, x + 1);
		if (first_value > 0 && second_value < 0 && horizontal_brightness.at<float>(0, x) > 50000.0f)
		{
			PointData new_point;
			new_point.position.x = x + (first_value/(first_value - second_value)); // compute with subpixel accuracy
			new_point.position.y = 0.0f;
			point_candidates.push_back(new_point);
		}
	}

	vertical_brightness = Mat(Size(1, gray_image.size().height), CV_32F);

	// for each candidate, y peak detection
	#pragma omp parallel for
	for (int i = 0; i < point_candidates.size(); i++)
	{
		vertical_brightness.setTo(0.0f);
		float max_vertical_value = 100.0f;

		for (int y = 0; y < gray_image.size().height; y++)
		{

			int center_x = static_cast<int>(point_candidates[i].position.x);

			if (center_x < 10){ center_x = 10;}
			if (center_x >= gray_image.size().height - 10) { center_x = gray_image.size().height - 11; }
			for (int x = center_x - 10; x < center_x + 10; x++)
			{
				vertical_brightness.at<float>(y, 0) += gray_image.at<unsigned char>(y, x);
			}
		}

		GaussianBlur(vertical_brightness, vertical_brightness, Size(1, 5), 0);
		Sobel(vertical_brightness, vertical_band_sobel, CV_32F, 0, 1, 5);

		for (int y = 0; y < gray_image.size().height - 1; y++)
		{
			float first_value = vertical_band_sobel.at<float>(y, 0);
			float second_value = vertical_band_sobel.at<float>(y + 1, 0);
			if (first_value > 0 && second_value < 0 && vertical_brightness.at<float>(y, 0) > max_vertical_value)
			{
				max_vertical_value = vertical_brightness.at<float>(y, 0);
				point_candidates[i].position.y = y + (first_value / (first_value - second_value)); // compute with subpixel accuracy
			}
		}

//		printf("Point candidate x:%f y:%f\n", point_candidates[i].position.x, point_candidates[i].position.y);
	}

	inout_points = point_candidates;

/*
	Mat vertical_brightness_disp = Mat(Size(gray_image.size().width, gray_image.size().height), CV_32F);

	// for debug display
	for (int x = 0; x < gray_image.size().width; x++)
	{
		for (int y = 0; y < gray_image.size().height; y++)
		{
			vertical_brightness_disp.at<float>(y, x) = vertical_brightness.at<float>(y, 0);
		}
	}

	cv::imshow("vertical_brightness_disp", vertical_brightness_disp * 0.001f);


	Mat horizontal_brightness_disp = Mat(Size(gray_image.size().width, gray_image.size().height), CV_32F);

	// for debug display
	for (int x = 0; x < gray_image.size().width; x++)
	{
		for (int y = 0; y < gray_image.size().height; y++)
		{
			horizontal_brightness_disp.at<float>(y, x) = horizontal_sobel.at<float>(0, x);
		}
	}

	cv::imshow("horizontal", horizontal_brightness_disp * 0.00001f);
*/
}

#ifdef __CODE_LED_TEST__

#include "Flea3.h"

// Unit test
int main()
{
	int video_mode = 0;
	VideoCapture video_reader;
	bool is_record = true;
	Flea3* camera_capture = NULL;
	
	VideoWriter video_writer;
	
	Mat color_mat;

	if(video_mode == 0)
	{ 
		camera_capture = new Flea3(0);
		camera_capture->SetShutter(0.031f);
		camera_capture->SetExposure(2.4f);
		camera_capture->SetGamma(4.0f);
		camera_capture->SetGain(18.1f);

		if(is_record)
			video_writer = VideoWriter(".\\Decoded.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 60.0, Size(1280, 960));

//		camera_capture.open(0);
//		camera_capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
//		camera_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 960);
//		camera_capture.set(CV_CAP_PROP_EXPOSURE, -12.0f);
//		camera_capture.set(CV_CAP_PROP_GAIN, 300.0f);
	}
	else if(video_mode == 1)
	{
		video_reader = VideoCapture(".\\OutVideo1.avi");
	}
	else
		color_mat = imread("debugimage.png");

	cv::Mat input_mat;
	cv::Mat output_mat;
	cv::Mat save_mat;

	if(video_mode == 0)
//		camera_capture >> input_mat;
		input_mat = camera_capture->Run();
	else if(video_mode == 1)
	{
		video_reader >> color_mat; // = cv::imread("rawimage1.png");
		cvtColor(color_mat, input_mat, CV_BGR2GRAY);
	}
	else
	{
		cvtColor(color_mat, input_mat, CV_BGR2GRAY);
	}

	CodeLED x_detector;

	x_detector.Initialize(input_mat);
	std::vector<PointData> point_data;

	int current_index = 0;

	while (true)
	{
		if(video_mode == 0)
			input_mat = camera_capture->Run();
		else if(video_mode == 1)
		{
			video_reader >> color_mat;
			if(color_mat.cols == 0) break;
			cvtColor(color_mat, input_mat, CV_BGR2GRAY);
		}
		else
			cvtColor(color_mat, input_mat, CV_BGR2GRAY);


		if(video_mode == 0)
		{ 
			cvtColor(input_mat, color_mat, CV_GRAY2BGR);
//			video_writer << color_mat;
		}

		cvtColor(input_mat, output_mat, CV_GRAY2BGR);

		int input_key = cv::waitKey(1);
		if (input_key == ' ') break;

		x_detector.Run(input_mat, point_data);
		cv::imshow("input", input_mat);

		for (int i = 0; i < point_data.size(); i++)
		{
			char buffer[10];
			cv::rectangle(output_mat, cv::Point(point_data[i].position.x - 20.0f, point_data[i].position.y - 20.0f), cv::Point(point_data[i].position.x + 20.0f, point_data[i].position.y + 20.0f), cv::Scalar(0, 200, 0), 5, 8);
			cv::putText(output_mat, _itoa(point_data[i].id, buffer, 10), cv::Point(point_data[i].position.x, point_data[i].position.y - 50.0f), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 200, 0), 2, CV_AA);
			printf("Point X:%.2f Y:%.2f DURATION:%d ID:%d\n", point_data[i].position.x, point_data[i].position.y, point_data[i].duration, point_data[i].id);
		}

		if (point_data.size() > 0 && video_mode == 1)
			if(point_data[0].id == -1)
				cv::imwrite("debugimage.png", color_mat);

		cv::imshow("output", output_mat);

		if(is_record)
			video_writer << output_mat;

		cv::waitKey(1);
/*
		output_mat.setTo(cv::Scalar(0, 0, 0));
		input_mat.copyTo(visual_mat[current_index]);
		for (int i = 0; i < 10; i++)
		{
			output_mat = output_mat + visual_mat[i];
		}
		current_index = (current_index + 1) % 10;

		cv::imshow("accum out", output_mat);
*/
	}

//	if(video_mode)
//		cv::imwrite("rawimage.png", save_mat);

	delete camera_capture;

	waitKey(0);

	return 0;
}

#endif // __CODE_LED_TEST__
