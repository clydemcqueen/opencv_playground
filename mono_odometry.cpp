#include <iostream>

#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// Some constants TODO
cv::TermCriteria term_crit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
cv::Size sub_pix_win_size(10,10), win_size(31,31);
const int MAX_COUNT = 500;

void findFeatures(cv::Mat &image, std::vector<cv::Point2f> &points)
{
  // Find Shi-Tomasi features
  goodFeaturesToTrack(image, points, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);

  // Compute subpixel locations
  cornerSubPix(image, points, sub_pix_win_size, cv::Size(-1,-1), term_crit);

  // If we found too few features, drop this frame
  if (points.size() < 20)
  {
    std::cout << "Found " << points.size() << " features; dropping this frame" << std::endl;
    points.clear();
  }
}

static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
{
}

int main( int argc, char** argv )
{
  // Hack to direct annoying ffmpeg error "Invalid UE golomb code" into a file, along with all other errors
  //freopen("error.txt", "w", stderr);

  std::cout << "OpenCV version " << CV_VERSION << std::endl;

  // TODO
  cv::CommandLineParser parser(argc, argv, "{@input|0|}");
  std::string input = parser.get<std::string>("@input");
  cv::VideoCapture cap;
  if (input.size() == 1 && isdigit(input[0]))
  {
    std::cout << "Opening video device " << input[0] - '0' << std::endl;
    cap.open(input[0] - '0');
  }
  else
  {
    std::cout << "Opening file " << input << std::endl;
    cap.open(input);
  }

  if (!cap.isOpened())
  {
    std::cout << "Couldn't open CV capture\n";
    return 0;
  }

  // Create a window
  cv::namedWindow("LK Demo", 1);
  cv::setMouseCallback( "LK Demo", onMouse, 0 );

  cv::Mat gray, prev_gray, image, frame;
  std::vector<cv::Point2f> points, prev_points;
  for(;;)
  {
    // Get the next frame
    cap >> frame;
    if (frame.empty())
    {
      std::cout << "End of file" << std::endl;
      break;
    }

    // Create a copy of the frame TODO why do this? Am I not allowed to modify frame?
    frame.copyTo(image);

    // Create a grayscale version
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    if (prev_points.empty())
    {
      // Bootstrap
      findFeatures(gray, points);
      std::cout << "Bootstrapped " << points.size() << " points" << std::endl;
    }
    else
    {
      std::vector<uchar> status;
      std::vector<float> err;

      // Find the previous features on this frame, and compute optical flow
      cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_points, points, status, err, win_size, 3, term_crit, 0, 0.001);

      // How many features did we find?
      size_t new_size = 0;
      for (size_t i = 0; i < points.size(); i++)
      {
        if (status[i])
        {
          cv::circle(image, points[i], 3, cv::Scalar(0, 0, 255), -1, 8);
          points[new_size++] = points[i];

        }
      }
      points.resize(new_size);
      std::cout << "LK found " << points.size() << " points in this frame" << std::endl;

      // Display the features we found
      cv::imshow("LK Demo", image);

      // Required for imshow
      cv::waitKey(10);

      // Calculate odometry if we have enough features
      if (points.size() > 10)
      {
      }

      // If we're low on features, find some more
      if (points.size() < 200)
      {
        findFeatures(gray, points);
      }
    }

    // Swap
    cv::swap(gray, prev_gray);
    std::swap(prev_points, points);
  }

  return 0;
}
