#include <opencv2/opencv.hpp>
#include <iostream>
#include "ekf_shared.h"

// kalman.cpp with imu-type acceleration measurement
// Added Θ'' to state so that we can measure Θ'' w/ the imu
// Can't do that and use Θ'' as a control variable, so, there's no control input
// Since we're measuring Θ'' vs. Θ, the results are much worse than kalman.cpp
// Still linear, so a kf, not an ekf

// State: Θ, Θ', Θ''
constexpr int STATE_DIM = 3;

// Control inputs: none
constexpr int CONTROL_DIM = 0;

// Measurement inputs: Θ''
constexpr int MEASURE_DIM = 1;

constexpr int SLEEP_TIME = 100;
constexpr double DT = SLEEP_TIME / 1000.0;

int main(int argc, char **argv)
{
  // Output of the controller
  double target_acceleration = 0;

  cv::Mat img(500, 500, CV_8UC3);
  cv::KalmanFilter kalman(STATE_DIM, MEASURE_DIM, CONTROL_DIM, CV_64F);

  // Measurements are MEASURE_DIM x 1
  cv::Mat z_k = cv::Mat::zeros(MEASURE_DIM, 1, CV_64F);

  // Transition matrix F is STATE_DIM x STATE_DIM
  double F[] = {1, DT, 0.5 * DT * DT, 0, 1, DT, 0, 0, 1};
  kalman.transitionMatrix = cv::Mat(STATE_DIM, STATE_DIM, CV_64F, F);

  // Measurement matrix H is MEASURE_DIM x STATE_DIM
  double H[] = {0, 0, 1};
  kalman.measurementMatrix = cv::Mat(MEASURE_DIM, STATE_DIM, CV_64F, H);

  // Process noise covariance Q is STATE_DIM x STATE_DIM
  cv::setIdentity(kalman.processNoiseCov, cv::Scalar(1e-5));

  // Measurement noise covariance R is MEASURE_DIM x MEASURE_DIM
  cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar(1e-3));

  // Posterior error covariance Σ is STATE_DIM x STATE_DIM
  cv::setIdentity(kalman.errorCovPost, cv::Scalar(1));

  // Initialize the posterior with a random state
  randn(kalman.statePost, 0.0, 0.1);

  for (;;)
  {
    // Project the model forward to time t + dt
    cv::Mat y_k = kalman.predict();

    // Generate a fake measurement using random noise
    cv::randn(z_k, 0.0, sqrt((double)kalman.measurementNoiseCov.at<double>(0, 0)));
    z_k.at<double>(0) += target_acceleration;

    // Plot points
    img = cv::Scalar::all(0);
    cv::circle(img, phi2xy(img, z_k), 4, cv::Scalar(128, 255, 255));    // Observed: yellow TODO what's a better way to display this?
    cv::circle(img, phi2xy(img, y_k), 4, cv::Scalar(255, 255, 255), 2); // Predicted: white bold
    cv::imshow("Kalman", img);
    printf("angle %g, velo %g, accel %g, target %g, imu %g\n",
      kalman.statePre.at<double>(0), kalman.statePre.at<double>(1), kalman.statePre.at<double>(2),
      target_acceleration, z_k.at<double>(0));

    // Correct
    kalman.correct(z_k);

    int key = cv::waitKey(SLEEP_TIME) & 255;
    switch (key)
    {
      case '+':
        printf("accelerate\n");
        target_acceleration += 0.01;
        break;
      case '-':
        printf("decelerate\n");
        target_acceleration -= 0.01;
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