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

// ShapeOp
#include "Solver.h"
#include "Constraint.h"
#include "Force.h"


using namespace std;

class Model {


public:
  Eigen::MatrixXd V;
  Eigen::MatrixXi E;
  Eigen::MatrixXi F;

  Eigen::MatrixXd Vel;
  Eigen::MatrixXd Force;
  Eigen::MatrixXd L0;

  ShapeOp::Solver* solver;

  double k_s;   // k_stress
  double k_e;   // k_electrostatic
  double h;

  Model(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

  void step();


};


#endif //GEODESY_MODEL_H
