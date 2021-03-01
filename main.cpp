#include <iostream>
#include <algorithm>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Dense>

// Mylar
#include "Model.h"
#include "Preprocessor.h"

using namespace std;

int main(int argc, char **argv) {
  Preprocessor p = Preprocessor();
  p.test();

  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);
  viewer.core().is_animating = true;
  viewer.core().background_color = Eigen::Vector4f(0.9, 0.9, 0.9, 1.0);

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  string input_dir = argv[1];
  igl::readOBJ(input_dir, V, F);

  auto* model = new Model(V, F, &viewer);
  viewer.data().set_mesh(model->V, model->F);
  Eigen::MatrixXd C = V * 0;
  for (int i=0; i<C.rows(); i++) {
    C(i,0) = 0.6;
    C(i,1) = 0.6;
    C(i,2) = 0.8;
  }
  viewer.data().set_colors(C);

  viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool {
    model->step();
    viewer.data().set_mesh(model->V,model->F);
    return true;
  };

  menu.callback_draw_viewer_menu =[&]()
  {
    ImGui::InputDouble("k_s", &model->k_s);
    ImGui::InputDouble("k_e", &model->k_e);
    ImGui::InputDouble("k_b", &model->k_b);
    ImGui::InputDouble("damping", &model->damping);
    ImGui::InputDouble("dampingFactor", &model->dampingFactor);
    ImGui::InputDouble("h", &model->h);

    if (ImGui::Button("begin")) {
      model->paused = false;
    }

    if (ImGui::Button("paused")) {
      model->paused = true;
    }

    if (ImGui::Button("reset")) {
      model = new Model(V, F, &viewer);
    }

  };

  viewer.launch();
}
