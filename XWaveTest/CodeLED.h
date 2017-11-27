#ifndef __CODE_LED__
#define __CODE_LED__

#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

using namespace cv;

struct PointData
{
	Point2f	position;
	int		id;
	int		duration;

	PointData() { position = Point2f(0.0f, 0.0f); id = -1; duration = 0; }
};

class CodeLED
{
protected:
	// setting values
	int brightness_threshold;
	int min_paint_threshold;
	int max_paint_threshold;
	float peak_threshold;
	float min_diff_threshold;
	int code_interval;

	int window_width;
	int window_height;

	int code_threshold[2];

	int decode_length_threshold;

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
	void DecodeID(PointData& inout_points);		// for gap length coding
	void DecodeID2(PointData& inout_points);	// for Manchester coding

public:
	void Initialize(Mat in_image);
	void Run(Mat in_image, std::vector<PointData>& out_points);
	void Finalize(){};
};


#endif