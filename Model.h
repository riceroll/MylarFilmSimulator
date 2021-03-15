//
// Created by 莱洛 on 2/15/21.
//

#ifndef GEODESY_MODEL_H
#define GEODESY_MODEL_H

#include <iostream>
#include <set>
#include <vector>
#include <tuple>

#include <Eigen/Core>
//#include "tqdm/tqdm.h"

#include <igl/opengl/glfw/Viewer.h>


using namespace std;

class Model {

public:
  Eigen::MatrixXd V;
  Eigen::MatrixXi E;
  Eigen::MatrixXi F;

  Eigen::MatrixXd Vel;
  Eigen::MatrixXd Acc;
  Eigen::MatrixXd Force;
  Eigen::MatrixXd L0;
  Eigen::VectorXd M; // mass vector

  igl::opengl::glfw::Viewer* viewer;

  double k_s;   // k_stress
  double k_e;   // k_electrostatic
  double k_b;   // k_bending
  double h;
  double damping;
  double damping_coeff;
  bool paused;

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

  Model(Eigen::MatrixXd& V, Eigen::MatrixXi& F, igl::opengl::glfw::Viewer* viewer);
  Model();

  void step();


};


#endif //GEODESY_MODEL_H
