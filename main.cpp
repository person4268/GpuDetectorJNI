#include "frc971/orin/971apriltag.h"
#include "apriltag.h"
#include "tag36h11.h"
#include <opencv2/opencv.hpp>
#include <iostream>

// pulled from a (very wrong) 1280x800 ov9281
frc971::apriltag::CameraMatrix intrinsics {
  .fx = 737.6136442454854,
  .cx = 662.3371068271363,
  .fy = 733.1927575565593,
  .cy = 435.9984845786
};

frc971::apriltag::DistCoeffs distortion {
  .k1 = 0.15288116557227518,
  .k2 = -0.2878953642242236,
  .p1 = -0.0010986978034486703,
  .p2 = 0.0011333394853758716,
  .k3 = 0.12276685039910991
};

int main(int argc, char **argv) {
  size_t decimate = 1;

  cv::Mat image;
  if(argc < 2) goto funny; // intentionally bad code
  image = cv::imread(argv[1]);
  if (image.empty()) {
    funny:
    std::cerr << "tf you doin bruh" << std::endl;
    return -1;
  }

  cv::Mat greyImg;
  cv::cvtColor(image, greyImg, cv::COLOR_BGR2GRAY);
  cv::Mat testImg;
  // cv::cvtColor(greyImg, testImg, cv::COLOR_GRAY2RGB);
  std::cout << "color size: " << image.elemSize() << " grey size: " << greyImg.elemSize() << std::endl;

  apriltag_detector_t *tag_detector = apriltag_detector_create();
  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_add_family(tag_detector, tag_family); //jni uses add_family_bits with bits_corrected = 1 (default is 2)
  tag_detector->quad_decimate = decimate;
  // original also set threads but that isnt used afaict?
  tag_detector->nthreads = 6;
  tag_detector->wp = workerpool_create(tag_detector->nthreads); // top 10 ways to get segfaults: ~~forgetting~~removing this line
  tag_detector->qtp.min_white_black_diff = 5; // idk but this was in the original code
  


  frc971::apriltag::GpuDetector detector(image.size[0], image.size[1], tag_detector, intrinsics, distortion, decimate);
  detector.DetectGrayHost(greyImg.data); // doesn't have a decimate==2 requirement. regular DetectGray crashes??
  // detector.DetectColor(image.data); // requires decimate 2?
  const zarray_t* detections = detector.Detections();
  std::cout << zarray_size(detections) << " detections" << std::endl;
  // why free pointers when you can just exit
}