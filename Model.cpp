//
// Created by 莱洛 on 2/15/21.
//

#include "Model.h"

Model::Model(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
  this->k_s = 10;
  this->k_e = 0.01;
  this->h = 0.2;

  this->V = V;
  this->V += Eigen::MatrixXd::Random(V.rows(), V.cols()) * 0.01;
  this->F = F;

  this->solver = new ShapeOp::Solver();

  set<tuple<int, int>> E;
  auto add_edge = [&](int i, int j) {
    if (E.count(make_tuple(i, j)) != 1 and E.count(make_tuple(j, i)) != 1 ) {
      E.insert(make_tuple(i, j));
    }
  };
  for (int i=0; i<this->F.rows(); i++) {
    Eigen::RowVector3i face = this->F.row(i);
    add_edge(face(0), face(1));
    add_edge(face(1), face(2));
    add_edge(face(2), face(0));
  }

  this->E.resize(E.size(), 2);

  int ie = 0;
  for (auto e : E) {
    this->E.row(ie) << get<0>(e), get<1>(e);
    ie++;
  }

  this->Vel = this->V * 0;
  this->Force = this->V * 0;
  this->L0.resize(this->E.rows(), 1);

  for (int i=0; i<this->E.rows(); i++) {
    auto v0 = this->V.row(this->E(i, 0));
    auto v1 = this->V.row(this->E(i, 1));

    double l = (v0-v1).norm();

    this->L0(i) = l;
  }

  // assembly
  vector<set<int>> face_sets;
  for (int i=0; i<this->F.rows(); i++) {
    set<int> face;
    face.emplace(this->F(i, 0));
    face.emplace(this->F(i, 1));
    face.emplace(this->F(i, 2));
    face_sets.push_back(face);
  }

  auto find_third_vertex = [&](int iv_i, int iv_j) {
    for (auto face : face_sets) {
      if (face.count(iv_i) and face.count(iv_j)) {
//        face.erase()
//        face.erase(iv_i);
//        face.erase(iv_j);
      }
    }

  };


  for (int i=0; i< this->E.rows(); i++) {
    int iv_i = this->E(i,0);
    int iv_j = this->E(i, 1);






  }





}

void Model::step() {

  this->Force *= 0;

  // edge force
  for (int i=0; i<this->E.rows(); i++) {
    auto iv0 = this->E(i, 0);
    auto iv1 = this->E(i, 1);
    auto v0 = this->V.row(iv0);
    auto v1 = this->V.row(iv1);

    double l = (v0-v1).norm();
    double f_edge = (l - this->L0(i)) * this->k_s;  // compressive_stress

    Eigen::RowVector3d f = (v1 - v0).normalized() * f_edge;

    this->Force.row(iv0) += f;
    this->Force.row(iv1) -= f;
  }

  // electrostatic force
  for (int i=0; i<this->V.rows(); i++) {
    for (int j=0; j<this->V.rows(); j++) {
      if (i == j) continue;

      auto v0 = this->V.row(i);
      auto v1 = this->V.row(j);
      double d = (v0-v1).norm();
      double q0 = 1;
      double q1 = 1;
      double f_electrostatic = q0 * q1 * this->k_e / d / d;

      Eigen::RowVector3d f = (v1 - v0).normalized() * f_electrostatic;

      this->Force.row(i) -= f;
      this->Force.row(j) += f;
    }
  }

  this->Vel += this->Force * this->h;
  this->V += this->Vel * this->h;

  // electrostatic force

}
