#include <stdio.h>
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

using namespace cv;

struct PointData
{
	Point2f	position;
	int		id;

	PointData(){position = Point2f(0.0f, 0.0f); id = -1; }
};

class XWaveDetector
{
protected:
	// setting values
	int brightness_threshold;
	int min_paint_threshold;
	int max_paint_threshold;
	float peak_threshold;

	int window_width;
	int window_height;

	int code_threshold[2];

	// for cog detection
	Mat rgb_image;
	Mat gray_image;
	Mat blur_image;
	Mat thresh_image;
	Mat labels_image;
	Mat mask_image;

	// for roi
	Mat roi_image;
	Mat roi_labels_image;

	// for Decode
	Mat sum_image;
	Mat sobel_image;
	Mat barcode_image;
	Mat dft_image;

	Mat element_erode;
	Mat element_dilate;

	int	 DynamicThresholding();
	void DetectCenter(int in_peak_threshold, int in_min_area_threshold, int in_max_area_threshold, std::vector<PointData>& inout_points);
	void SetROI(int in_width, int in_height, Point2f in_position);
	void GenerateBarcode();
	void DecodeID(PointData& inout_points);
public:
	void Initialize(Mat in_image);
	void Run(Mat in_image, std::vector<PointData>& out_points);
	void Finalize();
};


void XWaveDetector::Initialize(Mat in_image)
{
	// initialize setting values
	window_width = 512;
	window_height = 30;
	brightness_threshold = 150;
	min_paint_threshold = 10;
	max_paint_threshold = 500;
	peak_threshold = 80.0f;

	code_threshold[0] = 8;
	code_threshold[1] = 15;

	// for cog detection
	element_erode = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(3, 3));
	element_dilate = cv::getStructuringElement(CV_SHAPE_ELLIPSE, cv::Size(11, 11));

	labels_image = Mat(in_image.size(), CV_16U);

	// for peak detection
	sum_image = Mat(Size(window_width, window_height), CV_32F);
	sobel_image = cv::Mat(Size(window_width, window_height), CV_32F);
	barcode_image = cv::Mat(Size(window_width, window_height), CV_8U);
}

int XWaveDetector::DynamicThresholding()
{
	int max_value = 0;
	for (int i = 0; i < gray_image.size().width * gray_image.size().height; i++) {
		if(max_value < gray_image.data[i]){ max_value = gray_image.data[i]; }
	}

	return max_value - 80;
}

void XWaveDetector::Run(Mat in_image, std::vector<PointData>& out_points)
{
	cvtColor(in_image, gray_image, CV_BGRA2GRAY);	// Should be only 8bit or more...?

	brightness_threshold = DynamicThresholding();
	printf("Threshold: %d\n", brightness_threshold);
	DetectCenter(brightness_threshold, min_paint_threshold, max_paint_threshold, out_points);
	for (int i = 0; i < out_points.size(); i++)
	{
		SetROI(window_width, window_height, out_points[i].position);
		GenerateBarcode();
		DecodeID(out_points[i]);
		printf("id:%d x:%f, y:%f\n", out_points[i].id, out_points[i].position.x, out_points[i].position.y);
		cv::imshow("ROI", roi_image);
		cv::imshow("Barcode", barcode_image);
	}
}

void XWaveDetector::DecodeID(PointData& inout_point)
{
	std::vector<int>	       diff_width;
	std::vector<unsigned char> decode_array;
	bool is_up = false;
	int  diff_pos = window_width - 1;

	// signal length measurement
	for(int x=window_width - 1; x>=0; x--)
	{ 
		if (barcode_image.at<unsigned char>(0, x) > 0)
		{
			if(!is_up)
			{
				diff_width.push_back((x - diff_pos));
				diff_pos = x;	
			}

			is_up = true;
		}
		else
		{
			if (is_up) 
			{
				diff_width.push_back(-(x - diff_pos));
				diff_pos = x;
			}

			is_up = false;
		}
	}

	// decode signal length to binary
	for(int i=0; i<diff_width.size(); i++)
	{ 
		if (code_threshold[0] >=abs(diff_width[i])){  decode_array.push_back(0); }
		else if (code_threshold[0] < abs(diff_width[i]) && code_threshold[1] >= abs(diff_width[i])) { decode_array.push_back(1); }
		else if (code_threshold[1] < abs(diff_width[i])) { decode_array.push_back(2); }	// used as header code
	}

	// decode binary to ID
	bool is_head = false;
	int digit = 0;
	unsigned char decode_id = 0;

	for (int i = 0; i < decode_array.size(); i++)
	{
		if(decode_array[i] == 2){	digit = 0;	is_head = true;	}
		
		if(digit > 0 && digit <= 8 && is_head){ 
			decode_id = decode_id | decode_array[i] << (8 - digit);	
		}
		else if(digit == 9 && is_head)
		{	 
			int parity = ( (int)((decode_id >> 7) & 0b00000001)
						 + (int)((decode_id >> 6) & 0b00000001)
						 + (int)((decode_id >> 5) & 0b00000001)
						 + (int)((decode_id >> 4) & 0b00000001)
						 + (int)((decode_id >> 3) & 0b00000001)
						 + (int)((decode_id >> 2) & 0b00000001)
						 + (int)((decode_id >> 1) & 0b00000001)
						 + (int)((decode_id) & 0b00000001))
						 & 0b00000001;

			if(decode_array[i] == parity){	inout_point.id = (int)decode_id;	}
			is_head = false;
		}
		
		digit++;
	}

	// 
	FILE* fp = NULL; //fopen("Debug2.csv", "w");
	if (fp)
	{
		fprintf(fp, ", %d", diff_width[0]);

		for (int i = 1; i < diff_width.size(); i++) {
			fprintf(fp, ", %d", diff_width[i]);
		}
		fprintf(fp, "\n");

		fprintf(fp, ", %d", decode_array[0]);

		for (int i = 1; i < decode_array.size(); i++) {
			fprintf(fp, ", %d", decode_array[i]);
		}
		fprintf(fp, "\n");

		fclose(fp);
	}

}

void XWaveDetector::GenerateBarcode()
{
	float ave_value = 0.0f;

	for (int x = 0; x < window_width; x++)
	{
		float sum_value = 0.0f;
		int sum_count = 0;

		for (int y = 0; y < window_height; y++)
		{
			unsigned short label_value = roi_labels_image.at<unsigned short>(y, x);
			if (label_value == 0)
			{
				sum_value += (float)(roi_image.at<unsigned char>(y, x));
				sum_count++;
			}
		}

		if (sum_count > 0 && sum_count < window_height) {
			sum_value = sum_value * (float)roi_image.size().height / sum_count;
		}

		ave_value += sum_value;

		for (int y = 0; y < window_height; y++){
			sum_image.at<float>(y, x) = sum_value;
		}
	}

	ave_value /= window_width;

	cv::imshow("SUM", sum_image / 10000.0f);

	cv::GaussianBlur(sum_image, sum_image, cv::Size(5, 1), 0.0f);
	cv::Sobel(sum_image, sobel_image, CV_32F, 1, 0, 7);

	barcode_image.setTo(0);

	float peak_value = ave_value;
	float bottom_value = ave_value;
	bool  is_peak = false;
	bool  is_bottom = false;
	bool  is_up = false;

	float min_threshold_array[512];
	float max_threshold_array[512];

	for (int x = 0; x < roi_image.size().width - 1; x++)
	{
		float sobel_value1 = sobel_image.at<float>(0, x);
		float sobel_value2 = sobel_image.at<float>(0, x + 1);

		if (sobel_value1 > 0
		&& sobel_value2 < 0){
			float peak_rate = 1.0f - sobel_value1 / (sobel_value1 - sobel_value2);
			float new_peak = sum_image.at<float>(0, x) * peak_rate + sum_image.at<float>(0, x + 1) * (1.0f - peak_rate);

			if(new_peak - bottom_value > peak_threshold)
			{ 
				peak_value = new_peak;
				is_peak = true;
				is_up = true;

				if (is_peak
				&& is_bottom)
				{
					float middle_threshold = (peak_value + bottom_value) / 2.0f;
					int current_x = x - 1;

					while (middle_threshold < sum_image.at<float>(0, current_x))
					{
						barcode_image.at<unsigned char>(0, current_x) = 255;
						current_x--;
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
		else if (sobel_value1 < 0
		&& sobel_value2 > 0)
		{
			float peak_rate = 1.0f + sobel_value1 / (-sobel_value1 + sobel_value2);
			float new_bottom = sum_image.at<float>(0, x) * peak_rate + sum_image.at<float>(0, x + 1) * (1.0f - peak_rate);

			if(peak_value - new_bottom > peak_threshold)
			{ 
				bottom_value = new_bottom;
				is_bottom = true;
				is_up = false;

				if (is_peak
					&& is_bottom)
				{
					float middle_threshold = (peak_value + bottom_value) / 2.0f;
					int current_x = x - 1;
					while (middle_threshold > sum_image.at<float>(0, current_x))
					{
						barcode_image.at<unsigned char>(0, current_x) = 0;
						current_x--;
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

		max_threshold_array[x] = peak_value;
		min_threshold_array[x] = bottom_value;

		if(is_up)
		{ 
			barcode_image.at<unsigned char>(0, x) = 255;
		}
		else
		{
			barcode_image.at<unsigned char>(0, x) = 0;
		}
	}

	// for debug
	FILE* fp = NULL;//fopen("Debug.csv", "w");
	if (fp)
	{
		fprintf(fp, ", %f", sum_image.at<float>(0, 0));

		for (int x = 1; x < window_width; x++) {
			fprintf(fp, ", %f", sum_image.at<float>(0,x));
		}
		fprintf(fp, "\n");

		fprintf(fp, ", %d", barcode_image.at<unsigned char>(0, 0));

		for (int x = 1; x < window_width; x++) {
			fprintf(fp, ", %d", barcode_image.at<unsigned char>(0, x));
		}
		fprintf(fp, "\n");

		fprintf(fp, ", %f", max_threshold_array[0]);

		for (int x = 1; x < window_width; x++) {
			fprintf(fp, ", %f", max_threshold_array[x]);
		}
		fprintf(fp, "\n");

		fprintf(fp, ", %f", min_threshold_array[0]);

		for (int x = 1; x < window_width; x++) {
			fprintf(fp, ", %f", min_threshold_array[x]);
		}
		fprintf(fp, "\n");

		fclose(fp);
	}

	// genrate barcode
	for (int x = 0; x < roi_image.size().width - 1; x++)
	{
		for (int y = 0; y < roi_image.size().height; y++)
		{
			barcode_image.at<unsigned char>(y, x) = barcode_image.at<unsigned char>(0, x);
		}
	}
}

void XWaveDetector::SetROI(int in_width, int in_height, Point2f in_position)
{
	int center_x = (int)in_position.x;
	int center_y = (int)in_position.y;

	int leftup_x = center_x - in_width / 2;
	int leftup_y = center_y - in_height / 2;

	if (leftup_x < 0) { leftup_x = 0; }
	if (leftup_y < 0) { leftup_y = 0; }

	if (leftup_x + in_width > gray_image.size().width - 1){ leftup_x = gray_image.size().width - in_width - 1;	}
	if (leftup_y + in_height > gray_image.size().height - 1) { leftup_y = gray_image.size().height - in_height - 1; }

	roi_image = Mat(gray_image, Rect(leftup_x, leftup_y, in_width, in_height));
	roi_labels_image = Mat(labels_image, Rect(leftup_x, leftup_y, in_width, in_height));
}

void XWaveDetector::DetectCenter(int in_peak_threshold, int in_min_area_threshold, int in_max_area_threshold, std::vector<PointData>& inout_points)
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

	int i = 0;

	for (int y = 0; y < labels_image.size().height; y++)
	{
		for (int x = 0; x < labels_image.size().width; x++)
		{
			unsigned short label_data = ((unsigned short*)labels_image.data)[i++];
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
			printf("label:%d num:%d x:%f, y:%f\n", j, label_counter[j], new_point_data.position.x, new_point_data.position.y);
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



int main()
{
	VideoCapture camera_capture(2);
	camera_capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	camera_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
	camera_capture.set(CV_CAP_PROP_EXPOSURE, -12.0f);
	camera_capture.set(CV_CAP_PROP_GAIN, 200.0f);

	cv::Mat input_mat, flip_mat;// cv::imread("rawimage.jpg");

	camera_capture >> input_mat;
	cv::flip(input_mat, flip_mat, 0);

	XWaveDetector x_detector;

	x_detector.Initialize(flip_mat);
	std::vector<PointData> point_data;

	while (true)
	{
		camera_capture >> input_mat;
		cv::flip(input_mat, flip_mat, 0);

		int input_key = cv::waitKey(1);
		if (input_key == ' ') break;

		x_detector.Run(flip_mat, point_data);
		cv::imshow("Original", flip_mat);
	}

	return 0;
}