//#include <opencv2/opencv.hpp>
//
//int main()
//{
//	cv::Size chessBoardSize(11, 8);
//	float squareSize = 20.0;
//	std::vector<cv::Point3f> objectCorners;
//	for (size_t row = 0; row < 9; row++)
//	{
//		for (size_t col = 0; col < 11; col++)
//		{
//			objectCorners.push_back(cv::Point3f(col*squareSize, row*squareSize, 0.0));
//		}
//	}
//
//	for (size_t imgIdx = 1; imgIdx < 30; imgIdx++)
//	{
//		std::stringstream ss;
//		ss << imgIdx;
//		std::string imageName = "Image_" + ss.str() + ".bmp";
//		cv::Mat image = cv::imread(imageName, 0);
//
//		if(!cv::findChessboardCorners(image,chessBoardSize,))
//
//	}
//
//	return 0;
//}