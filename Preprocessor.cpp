//
// Created by 莱洛 on 2021/2/26.
//

#include "Preprocessor.h"
#include <iostream>

Preprocessor::Preprocessor() {
  Eigen::MatrixXd V;
  Eigen::MatrixXi E;
  Eigen::MatrixXd H;
  std::string flags;
  Eigen::MatrixXd V2;
  Eigen::MatrixXi F2;

  V.resize(4,2);
  V << -1,-1, 1,-1, 1,1, -1,1;
  E.resize(4,2);
  E << 0,1, 1,2, 2,3, 3,0;
  H.resize(0,2);
  // Triangulate the interior
  igl::triangle::triangulate(V,E,H,"a0.01q",V2,F2);

  std::cout<<"hehe";



//  igl::triangle::triangulate(V, E, H, flags, V2, F2);





}

void Preprocessor::test() {std::cout<<"22222222";}
