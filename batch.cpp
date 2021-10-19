#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

// Mylar
#include "Model.h"
#include "Utils.h"

using namespace std;


int main(int argc, char **argv) {
  // initialize variables
  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd NF; //per-face normal

  char* foo;
  std::vector<int> ivs_fixed;
  for (int i=0; i<argc; i++) {
    if (i==0) continue;
    int iv_fixed = std::strtol(argv[i], &foo, 10);
    ivs_fixed.push_back(iv_fixed);
  }

  vector<double> ks_e;
  vector<double> ks_b;

  double k_e = 0.00625;
  for (int i=0; i<10; i++) {
    ks_e.push_back(k_e);
    k_e *= 2;
  }
  double k_b = 0.001;
  for (int i=0; i<10; i++) {
    ks_b.push_back(k_b);
    k_b *= 10;
  }

  auto* model = new Model();
  auto* param = new Param();

  double n = 0;
  for (double k_e : ks_e){
    for (double k_b : ks_b) {
      param->k_e = k_e;
      param->k_b = k_b;

      // load initial model
      loadModel(viewer, V, F, NF, model, ivs_fixed, param);
      model->paused = false;

      model->step(model->steps_per_frame * 100);

      char buff[100];
      sprintf(buff, "%s/data/download/rose_%f_%f.obj", ROOT_DIR, k_e, k_b);
      string output_dir = buff;
      cout<< n/(ks_e.size() * ks_b.size())<<endl;
      n+=1;
      igl::writeOBJ(output_dir, model->V, model->F);

    }
  }

}
