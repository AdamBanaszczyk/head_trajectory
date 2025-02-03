#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;

/*
    Reads the data from file.
    Each row should contain "timestamp","x","y","z","qx","qy","qz","qw"
    Return 2D matrix of data
*/
MatrixXd readCSV(const string &filename, int save_step = 1) {
  vector<vector<double>> data;
  ifstream file(filename);

  if (!file.is_open()) {
    std::cout << "File not found: " << filename << std::endl;
    exit(0);
  }

  string line;

  // ignore header
  getline(file, line);

  uint8_t idx = 0;
  while (getline(file, line)) {
    stringstream linestream(line);
    string cell;
    vector<double> rowdata;
    while (getline(linestream, cell, ',')) {
      if (cell.size() > 2 && cell[0] == '"') {
        cell = cell.substr(1, cell.size() - 2);
      }
      if (cell.size() > 2 && cell[cell.size() - 1] == '"') {
        cell = cell.substr(0, cell.size() - 1);
      }

      rowdata.push_back(stod(cell));
    }

    if (idx % save_step == 0) {
      data.push_back(rowdata);
    }
    idx++;
  }
  file.close();

  MatrixXd matrix(data.size(), data[0].size());
  for (size_t i = 0; i < data.size(); ++i) {
    matrix.row(i) = VectorXd::Map(&data[i][0], data[0].size());
  }

  return matrix;
}

/*
    Interpolated data passed as 1st parameter to match timestamps for data
   passed as 2nd param Linear for translations, slerp for quats
*/
void interpolateData(Eigen::MatrixXd &t_opti, Eigen::MatrixXd &t_unity,
                         Eigen::MatrixXd &t_opti_match,
                         Eigen::MatrixXd &t_unity_match,
                         double time_offset = 0.0) {

  // Ignoring a match if timestamps differ by more than 20 ms                          
  constexpr double timestamp_matching_threshold = 0.02;
  
  size_t opti_index = 0, matched_idx = 0;
  for (size_t i = 0; i < t_unity.rows(); ++i) {
    double unity_timestamp = t_unity(i, 0) - time_offset;

    // Looking for a matching timestamp
    double opti_timestamp = 0;
    for (; opti_index < t_opti.rows(); ++opti_index) {
      opti_timestamp = t_opti(opti_index, 0);
      if (opti_timestamp > unity_timestamp) {
        break;
      }
    }

    // Weak match
    if (std::fabs(opti_timestamp - unity_timestamp) > timestamp_matching_threshold) {
      //std::cout << "Weak match? " << std::endl;
      continue;
    }

    t_unity_match.row(matched_idx) = t_unity.row(i);

    // No interpolation is feasible at start or at the end
    if (opti_index == 0 || opti_index > t_opti.rows() - 1) {
      opti_index = min(opti_index, (size_t)t_opti.rows() - 1);

      t_opti_match.row(matched_idx) = t_opti.row(opti_index);
      matched_idx++;
    }
    // Linear interpolation for translation and slerp for orientation based on
    // timestamps
    else {
      double &opti_timestamp_pre = t_opti(opti_index - 1, 0);
      double &opti_timestamp_post = t_opti(opti_index, 0);

      double weight_pre = (unity_timestamp - opti_timestamp_pre) /
                          (opti_timestamp_post - opti_timestamp_pre);
      double weight_post = (opti_timestamp_post - unity_timestamp) /
                           (opti_timestamp_post - opti_timestamp_pre);

      // interpolating timestamp and translation
      t_opti_match.block<1, 4>(matched_idx, 0) =
          weight_post * t_opti.block<1, 4>(opti_index - 1, 0) +
          weight_pre * t_opti.block<1, 4>(opti_index, 0);

      // interpolating quaternion (w!, x, y, z)
      Eigen::Quaterniond q1(
          t_opti(opti_index - 1, 7), t_opti(opti_index - 1, 4),
          t_opti(opti_index - 1, 5), t_opti(opti_index - 1, 6));
      Eigen::Quaterniond q2(t_opti(opti_index, 7), t_opti(opti_index, 4),
                            t_opti(opti_index, 5), t_opti(opti_index, 6));

      Eigen::Quaterniond inter_q = q1.slerp(weight_pre, q2);

      t_opti_match(matched_idx, 4) = inter_q.x();
      t_opti_match(matched_idx, 5) = inter_q.y();
      t_opti_match(matched_idx, 6) = inter_q.z();
      t_opti_match(matched_idx, 7) = inter_q.w();
      matched_idx++;
    }
  }
  t_unity_match.conservativeResize(matched_idx, 8);
  t_opti_match.conservativeResize(matched_idx, 8);
}

/*
    Converts a single data entry (row) to AngleAxis and translations
*/
void eigen_row_to_matrices(Eigen::MatrixXd &mat, size_t row_id,
                           Eigen::AngleAxisd &rot, Eigen::Vector3d &trans) {
  Eigen::Quaterniond q(mat(row_id, 7), mat(row_id, 4), mat(row_id, 5),
                       mat(row_id, 6));
  rot = Eigen::AngleAxisd(q);
  trans = Eigen::Vector3d(mat(row_id, 1), mat(row_id, 2), mat(row_id, 3));
}

/*
    Converts a single data entry (row) to Affine transformation
*/
void eigen_row_to_affine(Eigen::MatrixXd &mat, size_t row_id,
                         Eigen::Affine3d &aff) {
  Eigen::AngleAxisd rot;
  Eigen::Vector3d trans;
  eigen_row_to_matrices(mat, row_id, rot, trans);

  aff = Eigen::Affine3d::Identity();
  aff.translation() = trans;
  aff.linear() = rot.toRotationMatrix();
}

/*
    Takes data in data entry format (2D matrix) and returns a vector of affine
   transformations
*/
void matrix_to_affine(Eigen::MatrixXd &mat, vector<Affine3d> &affines) {
  for (size_t i = 0; i < mat.rows(); i++) {
    Eigen::Affine3d aff_i;
    eigen_row_to_affine(mat, i, aff_i);
    affines.push_back(aff_i);
  }
}

/*
    Converts a vector of affine transformations to OpenCV
*/
void affine_to_cv(vector<Affine3d> &affine, std::vector<cv::Mat> &cv_r_opti,
                  std::vector<cv::Mat> &cv_t_opti,
                  const size_t &pose_selection_step) {
  // Preparing for HandEye
  for (size_t i = 0; i < affine.size(); i = i + pose_selection_step) {
    // Translation to opencv
    cv::Mat R;
    Eigen::Matrix3d rot33 = affine[i].linear().matrix();
    cv::eigen2cv(rot33, R);
    cv_r_opti.push_back(R);

    cv::Mat t = (cv::Mat_<double>(3, 1) << affine[i].translation().x(),
                 affine[i].translation().y(), affine[i].translation().z());
    cv_t_opti.push_back(t);
  }
}

/*
    Converts OpenCV result to the Affine transformation
*/
void cv_to_affine(cv::Mat cv_rot, cv::Mat cv_trans, Affine3d &aff) {
  Eigen::Matrix3d R;
  cv::cv2eigen(cv_rot, R);
  Eigen::Vector3d t;
  cv::cv2eigen(cv_trans, t);

  aff = Eigen::Affine3d::Identity();
  aff.translation() = t;
  aff.linear() = R;
}

int main(int argc, char* argv[]) {
  if (argc < 3 || argc > 4) {
      std::cout << "Example use: ./trajectory_alignment opti.csv unity.csv [--t_diff=<float>]" << std::endl;
      std::exit(0);
  }

  size_t pose_selection_step = 1;

  // Default value for t_diff
  float t_diff = 0.0;
  bool t_diff_set = false; // Track if t_diff was explicitly specified


  // Parse required arguments
  std::string file1 = argv[1]; // Optitrack file
  std::string file2 = argv[2]; // Unity file

  // Optional --t_diff parameter
  for (int i = 3; i < argc; ++i) { // Check additional arguments
    std::string arg = argv[i];
    if (arg.rfind("--t_diff=", 0) == 0) { // Check if the argument starts with "--t_diff="
      try {
        t_diff = std::stof(arg.substr(9)); // Extract and convert the value after "--t_diff="
        t_diff_set = true; // Mark t_diff as explicitly set

        } catch (const std::exception& e) {
          // std::cerr << "Invalid value for --t_diff: " << arg.substr(9) << std::endl;
          std::cerr << "\033[1;31mError:\033[0m Invalid value for --t_diff: " << arg.substr(9) << std::endl;
          return 1;
          }
    } else {
      // std::cerr << "Unknown argument: " << arg << std::endl;
      std::cerr << "\033[1;31mError:\033[0m Unknown argument: " << arg << std::endl;
      return 1;
    }
  }

  // Output parsed values
  std::cout << "\nOptitrack file: " << file1 << std::endl;
  std::cout << "Unity file: " << file2 << std::endl;
  std::cout << "Time difference (t_diff): " << t_diff << " seconds" << std::endl;

  // Warn if t_diff was not specified
  if (!t_diff_set) {
    // std::cerr << "Warning: --t_diff not specified. Defaulting to 0.0." << std::endl;
    std::cerr << "\033[1;33mWarning:\033[0m --t_diff not specified. Defaulting to 0.0." << std::endl;
  }

  MatrixXd t_opti, t_unity;

  // data in rows: "timestamp","x","y","z","qx","qy","qz","qw"
  t_opti = readCSV(file1);	  // can dilute entries for better performance
  t_unity = readCSV(file2, 5); // adjust number or unity rows - helps with execution time
    
  // Transform unity to right-handed using x axis -> x = -x, qx = -qx, qw = -qw
  t_unity.col(1) = -t_unity.col(1); // x
  t_unity.col(4) = -t_unity.col(4); // qx
  t_unity.col(7) = -t_unity.col(7); // qw

  std::cout << "Data has been read : " << std::endl;
  std::cout << "\t t_opti has " << t_opti.rows() << " rows" << std::endl;
  std::cout << "\t t_unity has " << t_unity.rows() << " rows" << std::endl;

  // Compute new opti matching data for unity at provided timestamps
  MatrixXd t_unity_match(t_unity.rows(), t_unity.cols());
  MatrixXd t_opti_match(t_unity.rows(), t_unity.cols());
  interpolateData(t_opti, t_unity, t_opti_match, t_unity_match,  t_diff);

  // Now, we should have data with matching sizes
  if (t_opti_match.rows() != t_unity_match.rows()) {
    cerr << "Trajectories have different dimensions!" << endl;
    return -1;
  }

  // Move data to Eigen::Affine matrices
  vector<Affine3d> eigen_opti, eigen_unity;
  matrix_to_affine(t_opti_match, eigen_opti);
  matrix_to_affine(t_unity_match, eigen_unity);

  // We switch from unity pose in unity CS to unity CS in unity pose to match opencv input
  vector<Affine3d> eigen_unity_inv;
  for (int i = 0; i < eigen_unity.size(); i++) {
    eigen_unity_inv.push_back(eigen_unity[i].inverse());
  }

  // Data to OpenCV
  std::vector<cv::Mat> cv_r_opti, cv_t_opti, cv_r_unity, cv_t_unity;
  affine_to_cv(eigen_opti, cv_r_opti, cv_t_opti, pose_selection_step);
  affine_to_cv(eigen_unity_inv, cv_r_unity, cv_t_unity, pose_selection_step);

  // HandEyeCalibrationMethod
  // (https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b)
  cv::Mat cv_r_out, cv_t_out;
  cv::calibrateHandEye(cv_r_opti, cv_t_opti, cv_r_unity, cv_t_unity, cv_r_out,
                       cv_t_out, cv::CALIB_HAND_EYE_DANIILIDIS);

  // Convert result to Affine
  Eigen::Affine3d unity2opti;
  cv_to_affine(cv_r_out, cv_t_out, unity2opti);
  std::cout << "unity2opti : " << std::endl << unity2opti.matrix() << std::endl;

  // Saving transformation
  std::ofstream save_stream("unity2opti.csv");
  if (!save_stream.is_open()) {
    std::cout << "Error opening unity2opti.csv" << std::endl;
    exit(0);
  }

  for (int idx_row = 0; idx_row < unity2opti.matrix().rows(); idx_row++ ) {
    for (int idx_col = 0; idx_col < unity2opti.matrix().cols(); idx_col++ ) {
      save_stream << unity2opti.matrix()(idx_row, idx_col);
      if (idx_col + 1 != unity2opti.matrix().cols()) {
        save_stream << ", "; 
      }
    }
    save_stream << std::endl;
  }
  save_stream.close();

  // Verify errors
  double avg_err_t = 0, avg_err_r = 0;
  for (size_t i = 0; i < eigen_unity.size(); i++) {

    // Compute poses w.r.t. to the first entry
    Eigen::Affine3d unity = eigen_unity[0].inverse() * eigen_unity[i];
    Eigen::Affine3d opti = eigen_opti[0].inverse() * eigen_opti[i];

    // Moving unity motion to optitrack coordinates to compare it
    Eigen::Affine3d unity_motion_in_opti =
        unity2opti * unity * unity2opti.inverse();

    // Difference in motion between two measurements
    Eigen::Affine3d diff = unity_motion_in_opti.inverse() * opti;
    double diff_t = diff.translation().norm();
    double diff_r = AngleAxisd(diff.linear()).angle();

    avg_err_t += diff_t;
    avg_err_r += diff_r * 180.0 / M_PI;
    //std::cout << "i: " << i << "\t\t" << diff_t << " m,  "
    //          << diff_r * 180.0 / M_PI << " deg" << std::endl;
  }

  std::cout << "\nAVG !! avg_err_t = " << avg_err_t / eigen_unity.size()
            << " m, avg_err_r = " << avg_err_r / eigen_unity.size() << " deg | based on "
            << eigen_unity.size() << " measurements" << std::endl;

  return 0;
}
