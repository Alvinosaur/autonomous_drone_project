#include "auton_drone/PoseEstimation.h"

using namespace Eigen;

class PoseCovOutput {


public:
  MatrixXf X;
  Matrix4d P;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void predict(PoseCovOutput *prev_state,
             Matrix4d &process_cov, PoseCovOutput *result);
void update(PoseCovOutput *pred, std::vector<TagDetection> &detections,
            PoseCovOutput *result);
void calc_state_meas_trans(std::vector<TagDetection> &detections,
                          Affine3d &c);
Matrix4d process_cov;
Matrix4d meas_noise_cov;


PoseCovOutput* estimate_pose(std::vector<TagDetection> &detections,
                              PoseCovOutput *prev_state) {
  PoseCovOutput* prediction = new PoseCovOutput;
  // PoseCovOutput *new_estimate = new PoseCovOutput;
  PoseCovOutput* new_estimate = new PoseCovOutput;
  predict(prev_state, process_cov, prediction);
  update(prediction, detections, new_estimate);
  return new_estimate;
}

void predict(PoseCovOutput *prev_state,
             MatrixXf &process_cov, PoseCovOutput *pred) {
  // Have previous pose be the new predicted pose, no motion model
  pred->X = prev_state->X;

  // P_k = P_k-1 + Q
  process_cov = Matrix4d::Identity();
  pred->P = prev_state->P + process_cov;
}

void update(PoseCovOutput *pred, std::vector<TagDetection> &detections,
            PoseCovOutput *result) {
  Affine3d c;
  Affine3d Z;
  MatrixXf S;
  MatrixXf K;
  tf::transformTFToEigen(detections[0].transform, Z);

  calc_state_meas_trans(detections, c);
  meas_noise_cov = Matrix4d::Identity();
  S = (c * pred->P * c.matrix().transpose() + meas_noise_cov);
  K = pred->P * c * S.inverse();
  // result->X = pred->X + K * (Z.matrix() - pred->X * c.matrix());
  // result->P = (1 - K * c) * pred->P;
}

void calc_state_meas_trans(std::vector<TagDetection> &detections,
                          Affine3d &c) {
  tf::transformTFToEigen(detections[0].global_pose, c);
  /*
  int dim = 3;
  int num_tags = 4;
  int i;
  Matrix<double, ;
  for (i = 0; i < length(detections); i++) {
    A
  */
}

/*
void store_translation(tf::Vector3 &src, Vector3f &dst) {
  dst
}

    std::cout << "ORIG: ";
      print_transform(tag_poses[i].transform);

      tf::Transform new_trans = tag_poses[i].global_pose *
          tag_poses[i].transform.inverse();

      std::cout << "REVERT: ";
      print_transform(new_trans.inverse()*tag_poses[i].global_pose);
*/


