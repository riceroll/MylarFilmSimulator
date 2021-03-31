//
// Created by 莱洛 on 2/15/21.
//

#include "Model.h"

Model::Model() {
}

Model::Model(Eigen::MatrixXd& V, Eigen::MatrixXi& F, igl::opengl::glfw::Viewer* viewer) {
  this->viewer =viewer;

  this->V = V;
  this->F = F;

  // pre-normalize
  this->V.col(0) = this->V.col(0).array() - this->V.col(0).mean();
  this->V.col(1) = -(this->V.col(1).array() - this->V.col(1).mean());
  this->V.col(2) = this->V.col(2).array() - this->V.col(2).mean();
  double x_max = max(abs(V.col(0).maxCoeff()), abs(V.col(0).minCoeff()));
  double y_max = max(abs(V.col(1).maxCoeff()), abs(V.col(1).minCoeff()));
  double z_max = max(abs(V.col(2).maxCoeff()), abs(V.col(2).minCoeff()));
  this->V /= max(max(x_max, y_max), z_max);
  this->V *= 50;
  this->V += Eigen::MatrixXd::Random(V.rows(), V.cols()) * 1e-5;

  set<tuple<int, int>> Edges;
  auto add_edge = [&](int i, int j) {
    if (not (Edges.count(make_tuple(i, j)) or Edges.count(make_tuple(j, i))) ) {
      Edges.insert(make_tuple(i, j));
    }
  };
  for (int i=0; i<this->F.rows(); i++) {
    Eigen::RowVector3i face = this->F.row(i);
    add_edge(face(0), face(1));
    add_edge(face(1), face(2));
    add_edge(face(2), face(0));
  }

  this->E.resize(Edges.size(), 2);

  int ie = 0;
  for (auto e : Edges) {
    this->E.row(ie) << get<0>(e), get<1>(e);
    ie++;
  }

  this->Vel = this->V * 0;
  this->Acc = this->V * 0;
  this->Force = this->V * 0;
  this->L0.resize(this->E.rows(), 1);

  cout<<"Computing initial lengths..."<<endl;
  for (int i=0; i<this->E.rows(); i++) {
    auto v0 = this->V.row(this->E(i, 0));
    auto v1 = this->V.row(this->E(i, 1));

    double l = (v0-v1).norm();

    this->L0(i) = l;
  }

  // pre-assembly

  // mass-matrix with barycentric coordinate
  cout<<"Computing mass matrix..."<<endl;
  Eigen::SparseMatrix<double> Mass;
  igl::massmatrix<Eigen::MatrixXd, Eigen::MatrixXi, double>(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, Mass);
  this->M = Mass.diagonal();

  // pre-assembly for hinge constraint
  {
    cout<<"Emplacing faces.."<<endl;
    vector<set<int>> face_sets;
    for (int i = 0; i < this->F.rows(); i++) {
      set<int> face;
      face.emplace(this->F(i, 0));
      face.emplace(this->F(i, 1));
      face.emplace(this->F(i, 2));
      face_sets.emplace_back(face);
    }

    auto find_third_vertex = [&](int iv_i, int iv_j) {
      set<int> third_vs;
      for (auto face : face_sets) {
        if (face.count(iv_i) and face.count(iv_j)) {
          face.erase(iv_i);
          face.erase(iv_j);
          third_vs.emplace(*face.begin());
        }
        if (third_vs.size() == 2) return third_vs;
      }
      return third_vs;
    };

    cout<<"Pre-assembing matrices..."<<endl;
    for (int i = 0; i < this->E.rows(); i++) {
      int iv_i = this->E(i, 0);
      int iv_j = this->E(i, 1);
      set<int> third_vs = find_third_vertex(iv_i, iv_j);
      if (third_vs.size() != 2) { // boundary edge
        continue;
      }

      auto it = third_vs.begin();
      int iv_k = *it;
      it++;
      int iv_l = *it;

      Eigen::RowVector3d vi = this->V.row(iv_i);
      Eigen::RowVector3d vj = this->V.row(iv_j);
      Eigen::RowVector3d vk = this->V.row(iv_k);
      Eigen::RowVector3d vl = this->V.row(iv_l);
      Eigen::RowVector3d vec_ij = vj - vi;
      Eigen::RowVector3d vec_ki = vi - vk;
      auto normal = vec_ki.cross(vec_ij);
      if (normal.z() <= 0) {
        swap(iv_k, iv_l);
      }

      this->ivs_i.emplace_back(iv_i);
      this->ivs_j.emplace_back(iv_j);
      this->ivs_k.emplace_back(iv_k);
      this->ivs_l.emplace_back(iv_l);
    }

    cout<<"Precomputing done."<<endl;
  }

}

void Model::step(int n) {

  if (this->paused) {
    return;
  }

  // hinge force
  auto cot = [&](double t) {
    return t / sqrt(1 - t * t);
  };

  for (int i_step = 0; i_step < n; i_step ++) {
    this->Force *= 0;

    // edge force
    for (int i = 0; i < this->E.rows(); i++) {
      auto iv0 = this->E(i, 0);
      auto iv1 = this->E(i, 1);
      auto v0 = this->V.row(iv0);
      auto v1 = this->V.row(iv1);

      double l = (v0 - v1).norm();
      double f_edge = (l - this->L0(i)) / this->L0(i) * this->k_s;  // compressive_stress

      Eigen::RowVector3d f = (v1 - v0).normalized() * f_edge;

      this->Force.row(iv0) += f;
      this->Force.row(iv1) -= f;
    }

    // bending force
    for (int i = 0; i < this->ivs_i.size(); i++) {
      Eigen::RowVector3d x_i = this->V.row(this->ivs_i[i]);
      Eigen::RowVector3d x_j = this->V.row(this->ivs_j[i]);
      Eigen::RowVector3d x_k = this->V.row(this->ivs_k[i]);
      Eigen::RowVector3d x_l = this->V.row(this->ivs_l[i]);
      Eigen::RowVector3d d_ij = x_j - x_i;
      Eigen::RowVector3d d_ik = x_k - x_i;
      Eigen::RowVector3d d_il = x_l - x_i;
      Eigen::RowVector3d d_jk = x_k - x_j;
      Eigen::RowVector3d d_jl = x_l - x_j;
      double cos_jik = (pow(d_ij.norm(), 2) + pow(d_jk.norm(), 2) - pow(d_ik.norm(), 2))
                       / (2 * d_ij.norm() * d_jk.norm());
      double cos_ijk = (pow(d_ij.norm(), 2) + pow(d_ik.norm(), 2) - pow(d_jk.norm(), 2))
                       / (2 * d_ij.norm() * d_ik.norm());
      double cos_jil = (pow(d_ij.norm(), 2) + pow(d_jl.norm(), 2) - pow(d_il.norm(), 2))
                       / (2 * d_ij.norm() * d_jl.norm());
      double cos_ijl = (pow(d_ij.norm(), 2) + pow(d_il.norm(), 2) - pow(d_jl.norm(), 2))
                       / (2 * d_ij.norm() * d_il.norm());
      double cot_jik = cot(cos_jik);
      double cot_ijk = cot(cos_ijk);
      double cot_jil = cot(cos_jil);
      double cot_ijl = cot(cos_ijl);

      Eigen::RowVector3d n_k = d_ij.cross(d_jk).normalized();
      double h_k = d_ij.cross(d_jk).norm() / d_ij.norm();
      Eigen::RowVector3d n_l = -d_ij.cross(d_il).normalized();
      double h_l = (-d_ij.cross(d_il)).norm() / d_ij.norm();

      double sign = n_k.cross(n_l).dot(d_ij);
      double dot = n_k.dot(n_l);
      if (dot >= 1) dot = 1 - 1e-6;
      double rho = -sign * acos(dot);

      Eigen::RowVector3d partial_k = n_k / h_k;
      Eigen::RowVector3d partial_l = n_l / h_l;
      Eigen::RowVector3d partial_i =
        -cot_jik / (cot_jik + cot_ijk) * n_k / h_k - cot_jil / (cot_jil + cot_ijl) * n_l / h_l;
      Eigen::RowVector3d partial_j =
        -cot_ijk / (cot_jik + cot_ijk) * n_k / h_k - cot_ijl / (cot_jil + cot_ijl) * n_l / h_l;

      Eigen::RowVector3d f_i = -this->k_b * rho * partial_i;
      Eigen::RowVector3d f_j = -this->k_b * rho * partial_j;
      Eigen::RowVector3d f_k = -this->k_b * rho * partial_k;
      Eigen::RowVector3d f_l = -this->k_b * rho * partial_l;

      this->Force.row(this->ivs_i[i]) += f_i;
      this->Force.row(this->ivs_j[i]) += f_j;
      this->Force.row(this->ivs_k[i]) += f_k;
      this->Force.row(this->ivs_l[i]) += f_l;
    }

    // electrostatic force
#pragma omp parallel for default(none)
    for (int i = 0; i < this->V.rows(); i++) {
      for (int j = 0; j < this->V.rows(); j++) {
        if (i == j) continue;

        Eigen::RowVector3d v0 = this->V.row(i);
        Eigen::RowVector3d v1 = this->V.row(j);
        double area0 = this->M(i);
        double area1 = this->M(j);

        double d = (v0 - v1).norm();
        double q0 = area0;
        double q1 = area1;
        double f_electrostatic = q0 * q1 * this->k_e / d / d;

        Eigen::RowVector3d f = (v1 - v0).normalized() * f_electrostatic;

        this->Force.row(i) -= f;
        this->Force.row(j) += f;
      }
    }

    // damping force
    for (int i = 0; i < this->V.rows(); i++) {
      this->Force.row(i) += -this->Vel.row(i) * this->Vel.row(i).norm() * this->damping_coeff;
      float norm = this->Force.row(i).norm();
      if (norm > 1) {
        this->Force.row(i) /= norm;
//        this->Force.row(i) *= 0.8;
      }
    }

    this->Vel += this->Force * this->h;

    this->Vel *= this->damping;

    this->V += this->Vel * this->h;


    double maxVel = 0;
    for (int i = 0; i < this->Vel.rows(); i++) {
      double vel = this->Vel.row(i).norm();
      if (vel > maxVel) maxVel = vel;
    }

//    if (maxVel < 0.4) {
//      this->k_s = 6;
//      this->h = 0.25;
//      this->k_b = 0.2;
//      this->k_e = 0.01;
//    } else {
//      this->k_s = 2;
//      this->k_e = 0.005;
//      this->k_b = 0.2;
//      this->h = 0.1;
//    }

  }



}

