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

#include <igl/opengl/glfw/Viewer.h>


using namespace std;

class Model {


public:
  Eigen::MatrixXd V;
  Eigen::MatrixXi E;
  Eigen::MatrixXi F;

  Eigen::MatrixXd Vel;
  Eigen::MatrixXd Force;
  Eigen::MatrixXd L0;

  igl::opengl::glfw::Viewer* viewer;

  double k_s;   // k_stress
  double k_e;   // k_electrostatic
  double k_b;   // k_bending
  double h;
  double damping;
  double dampingFactor;
  bool paused;

  // pre-assembled variables
  vector<int> ivs_i;
  vector<int> ivs_j;
  vector<int> ivs_k;
  vector<int> ivs_l;

  Model(Eigen::MatrixXd& V, Eigen::MatrixXi& F, igl::opengl::glfw::Viewer* viewer);

  void step();


};


#endif //GEODESY_MODEL_H
