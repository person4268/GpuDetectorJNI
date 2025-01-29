#include "frc971/orin/971apriltag.h"
#include "apriltag.h"
#include "tag36h11.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

// pulled from a (very wrong) 1280x800 ov9281
frc971::apriltag::CameraMatrix intrinsics {
  .fx = 737.6136442454854,
  .cx = 662.3371068271363,
  .fy = 733.1927575565593,
  .cy = 435.9984845786
};

frc971::apriltag::DistCoeffs distortion {
  .k1 = 0,
  .k2 = 0,
  .p1 = 0,
  .p2 = 0,
  .k3 = 0
};

int main(int argc, char **argv) {
  size_t decimate = 1;
  
  cv::Mat image;
  cv::VideoCapture cap(0, cv::CAP_V4L2);
  if (!cap.isOpened()) {
    std::cerr << "sad. no video for you" << std::endl;
    return -1;
  }

  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap.set(cv::CAP_PROP_FPS, 30);

  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  int fps = cap.get(cv::CAP_PROP_FPS);
  printf("%dx%d @ %dfps\n", width, height, fps);

  cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);

  apriltag_detector_t *tag_detector = apriltag_detector_create();
  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_add_family(tag_detector, tag_family); //jni uses add_family_bits with bits_corrected = 1 (default is 2)
  tag_detector->quad_decimate = decimate;

  tag_detector->nthreads = 6;
  tag_detector->debug = false;
  tag_detector->wp = workerpool_create(tag_detector->nthreads); // top 10 ways to get segfaults: ~~forgetting~~removing this line
  tag_detector->qtp.min_white_black_diff = 5; // idk but this was in the original code

  frc971::apriltag::GpuDetector detector(width, height, tag_detector, intrinsics, distortion, decimate);

  while (true) {
    cap >> image;
    if (image.empty()) {
      std::cerr << "sad. no frame for you" << std::endl;
      break;
    }

    cv::Mat greyImg, out;
    cv::cvtColor(image, greyImg, cv::COLOR_BGR2GRAY);
    cv::cvtColor(greyImg, out, cv::COLOR_GRAY2BGR);

    auto start = std::chrono::high_resolution_clock::now();
    detector.DetectGrayHost(greyImg.ptr());
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "det: " << elapsed.count() << "s" << std::endl;

    const zarray_t* detections = detector.Detections();
    
    for(int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      cv::line(out, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 255, 0), 2);
      cv::line(out, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
      cv::line(out, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 255, 0), 2);
      cv::line(out, cv::Point(det->p[3][0], det->p[3][1]), cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("frame", out);
    if (cv::waitKey(1) == 27) break;
  }

  
}