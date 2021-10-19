//
// Created by 莱洛 on 2/15/21.
//

#ifndef GEODESY_MODEL_H
#define GEODESY_MODEL_H

#include <iostream>
#include <set>
#include <vector>
#include <tuple>
#include <cmath>

#include "Solver.h"
#include "Constraint.h"
#include <Eigen/Core>
//#include "tqdm/tqdm.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/bounding_box.h>
#include <igl/boundary_loop.h>

using namespace std;

class Param {
public:
  double k_b = 100;
  double k_e = 0.1;
  double w_length = 1;
  double w_closeness = 0;
  double w_plane = 0;
  double w_platform = 0;
  int steps_per_frame = 10;
};


class Model {

public:
  Eigen::MatrixXd V;
  Eigen::MatrixXi E;
  Eigen::MatrixXi F;
  Eigen::MatrixXd V_platform;

  Eigen::MatrixXd Vel;
  Eigen::MatrixXd Acc;
  Eigen::MatrixXd Force;
  Eigen::MatrixXd L0;
  Eigen::VectorXd M; // mass vector

  igl::opengl::glfw::Viewer* viewer;
  ShapeOp::Solver solver;
  Eigen::MatrixXd solver_points;
  Param* param;

  double k_s = 1000;  // k_stress
  double k_e = 0.1;   // k_electrostatic
//  double k_b = 100;     // k_bending
  double h = 0.02;
  double damping = 0.99;
  double damping_coeff = 0.2;
  bool paused = true;
  int steps_per_frame = 10;

  float camera_radius = 5;
  float camera_rad = M_PI / 2;
  float rad_per_frame = 0.00;

  bool show_edge_force = false;
  bool show_electrostatic_force = false;
  bool show_bending_force_i =false;
  bool show_bending_force_j =false;
  bool show_bending_force_k =false;
  bool show_bending_force_l =false;

  // pre-assembled variables
  vector<int> ivs_i;
  vector<int> ivs_j;
  vector<int> ivs_k;
  vector<int> ivs_l;

  vector<int> ivs_fixed;

  Model(Eigen::MatrixXd& V, Eigen::MatrixXi& F, igl::opengl::glfw::Viewer* viewer, Param* param);
  Model();

  void step(int n);

  void step2(int n);


  Eigen::RowVector3d elec_force(Eigen::RowVector3d v0, Eigen::RowVector3d v1, double area0, double area1);

};

void loadModel(igl::opengl::glfw::Viewer& viewer,
               Eigen::MatrixXd& V,
               Eigen::MatrixXi& F,
               Eigen::MatrixXd& NF,
               Model* model,
               vector<int> ivs_fixed,
               Param* param);

#endif //GEODESY_MODEL_H
