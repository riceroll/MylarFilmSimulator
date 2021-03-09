#include <iostream>
#include <algorithm>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Dense>

// Mylar
#include "Model.h"
#include "Preprocessor.h"

using namespace std;

int main(int argc, char **argv) {
  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  viewer.plugins.push_back(&menu);
  viewer.core().is_animating = true;
  viewer.core().background_color = Eigen::Vector4f(0.9, 0.9, 0.9, 1.0);

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  char buff[100];
  sprintf(buff, "%s/data/out.obj", ROOT_DIR);
  string input_dir = buff;
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



  int num_choices = 0;
  menu.callback_draw_viewer_menu =[&]()
  {

//    if (ImGui::Button("Load"))
//    {
//      std::string input_dir = igl::file_dialog_open();
//      string root_dir = ROOT_DIR;
//      root_dir += '/';
//      Preprocessor p = Preprocessor(input_dir, root_dir);
//      igl::readOBJ(root_dir + "data/out.obj", V, F);
//      model = new Model(V, F, &viewer);
//      viewer.data().clear();
//      viewer.data().set_mesh(model->V, model->F);
//      C = V * 0;
//      for (int i=0; i<C.rows(); i++) {
//        C(i,0) = 0.6;
//        C(i,1) = 0.6;
//        C(i,2) = 0.8;
//      }
//      viewer.data().set_colors(C);
//    }

    ImGui::InputDouble("k_s", &model->k_s);
    ImGui::InputDouble("k_e", &model->k_e);
    ImGui::InputDouble("k_b", &model->k_b);
    ImGui::InputDouble("damping", &model->damping);
    ImGui::InputDouble("dampingFactor", &model->damping_coeff);
    ImGui::InputDouble("h", &model->h);
//    ImGui::Button("load") {
//    }

    if (ImGui::Button("begin")) {
      model->paused = false;
    }

    if (ImGui::Button("paused")) {
      model->paused = true;
    }

    if (ImGui::Button("reset")) {
      model = new Model(V, F, &viewer);
    }

    if (ImGui::InputInt("Num letters", &num_choices))
    {
      model->show_bending_force_i = false;
      model->show_bending_force_j = false;
      model->show_bending_force_k = false;
      model->show_bending_force_l = false;
      model->show_edge_force = false;
      model->show_electrostatic_force = false;


      switch(num_choices) {

        case 0 :
          model->show_bending_force_i = true;
          break;

        case 1 :
          model->show_bending_force_j = true;
          break;

        case 2 :
          model->show_bending_force_k = true;
          break;

        case 3 :
          model->show_bending_force_l = true;
          break;

        case 4 :
          model->show_electrostatic_force = true;
          break;

        case 5 :
          model->show_edge_force = true;
          break;

        default :
          cout<<"default"<<endl;
          break;

      }

    }



  };

  viewer.launch();
}
