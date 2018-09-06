#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
  cv::namedWindow("ExampleGray", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("ExampleCanny", cv::WINDOW_AUTOSIZE);

  cv::VideoCapture cap;
  cap.open(std::string(argv[1]));

  cv::Mat color, gray, canny;

  for (;;)
  {
    cap >> color;
    if (color.empty()) break;

    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    cv::imshow("ExampleGray", gray);

    cv::Canny(gray, canny, 10, 100, 3, true);
    cv::imshow("ExampleCanny", canny);

    if (cv::waitKey(1) >= 0) break;
  }

  return 0;
}