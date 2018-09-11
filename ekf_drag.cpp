#include <opencv2/opencv.hpp>
#include <iostream>

cv::Point2i phi2xy(cv::Mat& img, cv::Mat& angle)
{
  return cv::Point(
    cvRound(img.cols / 2 + img.cols / 3 * cos(angle.at<double>(0))),
    cvRound(img.rows / 2 + img.cols / 3 * sin(angle.at<double>(0))));
}

constexpr int SLEEP_TIME = 100;
constexpr double DT = SLEEP_TIME / 1000.0;

int main(int argc, char **argv)
{
  cv::Mat img(500, 500, CV_8UC3);
  cv::KalmanFilter kalman(2, 1, 1, CV_64F);

  // State is 2x1 (n=2) [angle, angular velocity]
  cv::Mat x_k = cv::Mat::zeros(2, 1, CV_64F);

  // Control is 1x1 (c=1) [angular acceleration]
  cv::Mat u_k = cv::Mat::zeros(1, 1, CV_64F);

  // Process noise is 2x1
  cv::Mat w_k(2, 1, CV_64F);

  // Measurements are 1x1 [angle]
  cv::Mat z_k = cv::Mat::zeros(1, 1, CV_64F);

  // Control matrix B is 2x1, set to [[0], [DT]]
  double B[] = {0, DT};
  kalman.controlMatrix = cv::Mat(2, 1, CV_64F, B).clone(); // Not sure why clone()?

  // Measurement matrix H is 2x1, set to [1, 0]
  cv::setIdentity(kalman.measurementMatrix, cv::Scalar(1));

  // Process noise covariance Q is 2x2, set to [[1e-5, 0], [0, 1e-5]]
  cv::setIdentity(kalman.processNoiseCov, cv::Scalar(1e-5));

  // Measurement noise covariance R is 1x1, set to [1e-1]
  cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar(1e-1));

  // Posterior error covariance Σ is 2x2, initialize to [[1, 0], [0, 1]]
  cv::setIdentity(kalman.errorCovPost, cv::Scalar(1));

  // Initialize the posterior with a random state
  randn(kalman.statePost, 0.0, 0.1);

  for (;;)
  {
    // Transition matrix F is 2x2, set to [[1, DT], [0, 1 - c * angular_velo]]
    // Compute (1 - c * angular_velo) for every cycle
    constexpr double DRAG_CONSTANT = 0.1;
    double velo_term = 1.0 - DRAG_CONSTANT * std::abs(x_k.at<double>(1));
    double F[] = {1, DT, 0, velo_term};
    kalman.transitionMatrix = cv::Mat(2, 2, CV_64F, F).clone(); // Not sure why clone()?

    // Prediction
    cv::Mat y_k = kalman.predict(u_k);

    // Generate a fake measurement using random noise
    cv::randn(z_k, 0.0, sqrt((double)kalman.measurementNoiseCov.at<double>(0, 0)));
    z_k = kalman.measurementMatrix * x_k + z_k;

    // Plot points
    img = cv::Scalar::all(0);
    cv::circle(img, phi2xy(img, z_k), 4, cv::Scalar(128, 255, 255));    // Observed: yellow
    cv::circle(img, phi2xy(img, y_k), 4, cv::Scalar(255, 255, 255), 2); // Predicted: white bold
    cv::circle(img, phi2xy(img, x_k), 4, cv::Scalar(0, 0, 255));        // Actual: red
    cv::imshow("Kalman", img);
    printf("angle %g, velo %g, accel %g\n", x_k.at<double>(0), x_k.at<double>(1), u_k.at<double>(0));

    // Adjust filter state
    kalman.correct(z_k);

    // Step: apply transition matrix and control matrix, and add some process noise
    cv::randn(w_k, 0.0, sqrt((double)kalman.processNoiseCov.at<double>(0, 0)));
    x_k = kalman.transitionMatrix * x_k + kalman.controlMatrix * u_k + w_k;

    int key = cv::waitKey(SLEEP_TIME) & 255;
    switch (key)
    {
      case '+':
        u_k.at<double>(0) += 0.01;
        printf("increase acceleration %g\n", u_k.at<double>(0));
        break;
      case '-':
        u_k.at<double>(0) -= 0.01;
        printf("decrease acceleration %g\n", u_k.at<double>(0));
        break;
      case '0':
        printf("coast\n");
        u_k.at<double>(0) = 0.0;
        break;
      case 27:
        return 0;
      default:
        break;
    }
  }

  return 0;
}