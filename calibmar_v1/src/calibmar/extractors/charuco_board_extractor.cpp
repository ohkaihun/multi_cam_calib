#include "calibmar/extractors/charuco_board_extractor.h"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
namespace {
  inline int MapCornerToPointId(int marker_id, int corner_idx) {
    return marker_id * 4 + corner_idx;
  }
}
namespace calibmar {              

  // void CharucoFeatureExtractor::Options::Check() {
  //   int limit = 3;
  //   if (chessboard_rows <= limit || chessboard_columns <= limit) {
  //     throw std::runtime_error("Too few chessboard rows or columns.");
  //   }
  //   if (square_size <= 0.0 || marker_size <= 0.0) {
  //     throw std::runtime_error("Square size and marker size must be greater than zero.");
  //   }
  // }

  CharucoBoardFeatureExtractor::CharucoBoardFeatureExtractor(const Options& options) : options_(options) {
    // generate 3D points for Charuco board
    uint32_t idx = 0;
    for (int row = 0; row < options_.rows-1; row++) {
      for (int col = 0; col < options_.columns-1; col++) {
        double x = col * options_.square_size;
        double y = row * options_.square_size;
        double z = 0.0;
        points3D_[idx] = Eigen::Vector3d(x, y, z);
        idx++;
      }
    }
  }
    // std::vector<Eigen::Vector3d> get3DPointsFromCharucoIds(const std::vector<int>& charucoIds) {
    //   std::vector<Eigen::Vector3d> points3D;
    //   points3D.reserve(charucoIds.size());

    //   for (int id : charucoIds) {
    //       points3D.push_back(points3D_[id]);
    //   }

    //   return points3D;
    // }
  FeatureExtractor::Status CharucoBoardFeatureExtractor::Extract(Image& image, const Pixmap& pixmap) {
    if (pixmap.Width() <= 0 || pixmap.Height() <= 0) {
      return Status::DETECTION_ERROR;
    }

    cv::Mat image_data(pixmap.Data());
    cv::Mat grayImage;
    cv::cvtColor(image_data, grayImage, cv::COLOR_BGR2GRAY);
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected_candidates;
    // std::vector<std::vector<cv::Point2f>> charucoCorners;
    cv::Mat charucoCorners;
    std::vector<int> charucoIds;

// static_cast<int>(options_.aruco_type
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(static_cast<int>(options_.aruco_type));
    std::vector<int> ids(options_.marker_num);
    for (int i = 0; i <options_.marker_num; i++) {
        ids[i] = options_.board_index * options_.marker_num + i;
    }
    cv::Ptr<cv::aruco::CharucoBoard> board = new cv::aruco::CharucoBoard(cv::Size(options_.columns ,options_.rows),
                                            options_.square_size, options_.marker_size, dictionary,ids);

    // board->setLegacyPattern(true);
    // cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(5, 7, 0.04, 0.02, dictionary);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    // params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;

    // cv::aruco::ArucoDetector detector(dictionary, *params);
    // detector.detectMarkers(image_data,marker_corners,marker_ids,rejected_candidates);

    cv::aruco::detectMarkers(image_data, cv::makePtr<cv::aruco::Dictionary>(board->getDictionary()), marker_corners, marker_ids, params,rejected_candidates);
    if (marker_corners.size()==0) {
      return Status::DETECTION_ERROR;
    }

    else { 
      std::vector<cv::Point2f> charucoCorners; 
      std::vector<int> charucoIds; 
      cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, grayImage, board,  charucoCorners, charucoIds);
      if (charucoCorners.size()<8) {
        return Status::LACK_ERROR;
      }


      std::map<int, size_t> id_to_idx;
      for (size_t i = 0; i < marker_ids.size(); i++) {
        // sort the points accoring to marker id, to keep stable detection for livestream
        id_to_idx[marker_ids[i]] = i;
      }

      // if (charucoCorners.size() != points3D_.size()) {
      //   return Status::DETECTION_ERROR;
      // }

      image.ClearCorrespondences();
      std::vector<Eigen::Vector2d> points2D;
      // std::vector<Eigen::Vector3d> points3D;
      for (size_t i = 0; i < charucoIds.size(); i++) {
        const cv::Point2f& corner = charucoCorners[i];
        // findChessboardCorners sets the image origin at center of the first pixel,
        // while for colmap camera models the center of the first pixel is at 0.5, 0.5.
        // Eigen::Vector3d point3D(corner.x + 0.5, corner.y + 0.5, charucoIds[i]);
        Eigen::Vector2d point2D(corner.x, corner.y);
        points2D.push_back(point2D);
        size_t idx = image.AddPoint2D(point2D);
        image.SetPoint3DforPoint2D(charucoIds[i], idx);
        // points3D.push_back(point3D);
      }
      image.SetPoints2D(points2D);
      // image.SetPoints3D(points3D);
      image.SetCharucoIds(charucoIds);

      std::map<int, std::vector<Eigen::Vector2d>> aruco_keypoints;
      // the corner sequence matches the 3D point sequence.
      // for (size_t i = 0; i < charucoIds.size(); i++) {
      //   image.SetPoint3DforPoint2D(i, i);
      // }

      return Status::SUCCESS;}
  }
}  // namespace calibmar