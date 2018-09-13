#include <opencv2/opencv.hpp>
#include <iostream>
#include "ekf_shared.h"

// kalman.cpp modified to support drag (friction proportional to velocity squared)
// Drag can be added to the control input, so F and B are still linear -- thus, this is still a kf

// State: Θ, Θ'
constexpr int STATE_DIM = 2;

// Control inputs: Θ''
constexpr int CONTROL_DIM = 1;

// Measurement inputs: Θ
constexpr int MEASURE_DIM = 1;

constexpr int SLEEP_TIME = 100;
constexpr double DT = SLEEP_TIME / 1000.0;

int main(int argc, char **argv)
{
  // Output of the controller
  double target_acceleration = 0;

  cv::Mat img(500, 500, CV_8UC3);
  cv::KalmanFilter kalman(STATE_DIM, MEASURE_DIM, CONTROL_DIM, CV_64F);

  // State is STATE_DIM x 1
  cv::Mat x_k = cv::Mat::zeros(STATE_DIM, 1, CV_64F);

  // Control is CONTROL_DIM x 1
  cv::Mat u_k = cv::Mat::zeros(CONTROL_DIM, 1, CV_64F);

  // Process noise is STATE_DIM x 1
  cv::Mat w_k(STATE_DIM, 1, CV_64F);

  // Measurements are MEASURE_DIM x 1
  cv::Mat z_k = cv::Mat::zeros(MEASURE_DIM, 1, CV_64F);

  // Transition matrix F is STATE_DIM x STATE_DIM
  double F[] = {1, DT, 0, 1};
  kalman.transitionMatrix = cv::Mat(STATE_DIM, STATE_DIM, CV_64F, F);

  // Control matrix B is STATE_DIM x CONTROL_DIM
  double B[] = {0, DT};
  kalman.controlMatrix = cv::Mat(STATE_DIM, CONTROL_DIM, CV_64F, B);

  // Measurement matrix H is MEASURE_DIM x STATE_DIM
  double H[] = {1, 0};
  kalman.measurementMatrix = cv::Mat(MEASURE_DIM, STATE_DIM, CV_64F, H);

  // Process noise covariance Q is STATE_DIM x STATE_DIM
  cv::setIdentity(kalman.processNoiseCov, cv::Scalar(1e-5));

  // Measurement noise covariance R is MEASURE_DIM x MEASURE_DIM
  cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar(1e-1));

  // Posterior error covariance Σ is STATE_DIM x STATE_DIM
  cv::setIdentity(kalman.errorCovPost, cv::Scalar(1));

  // Initialize the posterior with a random state
  randn(kalman.statePost, 0.0, 0.1);

  for (;;)
  {
    // Control input is target acceleration - acceleration due to drag
    constexpr double DRAG_CONSTANT = 0.1;
    u_k.at<double>(0) = target_acceleration - DRAG_CONSTANT * x_k.at<double>(1) * std::abs(x_k.at<double>(1));

    // Project the model forward to time t + dt
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
    printf("angle %g, velo %g, accel %g, target %g\n",
      x_k.at<double>(0), x_k.at<double>(1), u_k.at<double>(0), target_acceleration);

    // Adjust filter state
    kalman.correct(z_k);

    // Generate a sort of "ground truth": apply transition matrix and control matrix, and add some process noise
    cv::randn(w_k, 0.0, sqrt((double)kalman.processNoiseCov.at<double>(0, 0)));
    x_k = kalman.transitionMatrix * x_k + kalman.controlMatrix * u_k + w_k;

    int key = cv::waitKey(SLEEP_TIME) & 255;
    switch (key)
    {
      case '+':
        target_acceleration += 0.01;
        printf("increase acceleration %g\n", target_acceleration);
        break;
      case '-':
        target_acceleration -= 0.01;
        printf("decrease acceleration %g\n", target_acceleration);
        break;
      case '0':
        printf("coast\n");
        target_acceleration = 0.0;
        break;
      case 27:
        return 0;
      default:
        break;
    }
  }
}