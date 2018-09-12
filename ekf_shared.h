cv::Point2i phi2xy(cv::Mat& img, cv::Mat& angle)
{
  return cv::Point(
    cvRound(img.cols / 2 + img.cols / 3 * cos(angle.at<double>(0))),
    cvRound(img.rows / 2 + img.cols / 3 * sin(angle.at<double>(0))));
}

