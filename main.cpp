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
  bool debug = false;

  cv::Mat image;
  if(argc < 2) goto funny; // intentionally bad code
  image = cv::imread(argv[1]);
  if (image.empty()) {
    funny:
    std::cerr << "tf you doin bruh" << std::endl;
    return -1;
  }

  cv::Mat greyImg, tresholded, out;
  cv::cvtColor(image, greyImg, cv::COLOR_BGR2GRAY);
  greyImg.copyTo(tresholded); // lazy way to preallocate image of the same size lol
  cv::cvtColor(greyImg, out, cv::COLOR_GRAY2BGR);


  apriltag_detector_t *tag_detector = apriltag_detector_create();
  apriltag_family_t *tag_family = tag36h11_create();
  apriltag_detector_add_family(tag_detector, tag_family); //jni uses add_family_bits with bits_corrected = 1 (default is 2)
  tag_detector->quad_decimate = decimate;
  // original also set threads but that isnt used afaict?
  tag_detector->nthreads = 6;
  tag_detector->debug = false;
  tag_detector->wp = workerpool_create(tag_detector->nthreads); // top 10 ways to get segfaults: ~~forgetting~~removing this line
  tag_detector->qtp.min_white_black_diff = 5; // idk but this was in the original code
  


  std::cout << "image size: " << image.size[0] << "," << image.size[1] << std::endl;
  frc971::apriltag::GpuDetector detector(image.size[1], image.size[0], tag_detector, intrinsics, distortion, decimate);

  for(int j = 0; j < 3; j++) {
    auto start = std::chrono::high_resolution_clock::now();
    detector.DetectGrayHost(greyImg.ptr()); // doesn't have a decimate==2 requirement. regular DetectGray crashes??
    // detector.DetectColor(image.data); // requires decimate 2?
    const zarray_t* detections = detector.Detections();

    if(debug) {
      detector.CopyThresholdedTo(tresholded.ptr());
      cv::imwrite("thresholded.png", tresholded);
    
      auto fitQuads = detector.CopyFitQuads();
      std::cout << fitQuads.size() << " fit quads" << std::endl;
    }

    std::cout << zarray_size(detections) << " detections" << std::endl;
    if(debug) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        std::cout << "Detection " << i << ": " << det->id << std::endl;
        for (int j = 0; j < 4; j++) {
          std::cout << "Corner " << j << ": " << det->p[j][0] << "," << det->p[j][1] << std::endl;
        }

        // i wrote this a while ago tbh
        cv::line(out, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 0xff, 0), 2);
        cv::line(out, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 0, 0xff), 2);
        cv::line(out, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0xff, 0, 0), 2);
        cv::line(out, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0xff, 0, 0), 2);
      }

      cv::imwrite("output.png", out);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "detection took " << std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start).count() << "ms" << std::endl;
  }

  // why free pointers when you can just exit
}