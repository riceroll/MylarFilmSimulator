#include <iostream>
#include <fstream>
#include <algorithm>

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

// Mylar
#include "Model.h"
#include "Utils.h"

using namespace std;

// reload model from the same obj file
void loadModel(igl::opengl::glfw::Viewer& viewer,
               Eigen::MatrixXd& V,
               Eigen::MatrixXi& F,
               Eigen::MatrixXd& NF,
               Model* model) {

  char buff[100];
  sprintf(buff, "%s/data/out.obj", ROOT_DIR);
  string input_dir = buff;
  igl::readOBJ(input_dir, V, F);
  *model = Model(V, F, &viewer);
  viewer.data().clear();
  viewer.data().set_mesh(model->V, model->F);
  igl::per_face_normals(model->V, model->F,NF);
  viewer.data().set_normals(NF);
//  viewer.data().double_sided = true;

  Eigen::MatrixXd C = V * 0;
  for (int i=0; i<C.rows(); i++) {
    C(i,0) = 0.6;
    C(i,1) = 0.6;
    C(i,2) = 0.8;
  }
  viewer.data().set_colors(C);
}

int main(int argc, char **argv) {

  // initialize variables
  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiMenu menu;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  Eigen::MatrixXd NF; //per-face normal

  auto* model = new Model();
  auto* utils = new Utils();
  unsigned int last_modified_time = utils->last_modified_time("./data/timestamp");

  // load initial model
  loadModel(viewer, V, F, NF, model);

  // callback
  viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool {
    if (last_modified_time != utils->last_modified_time("./data/timestamp")) {
      std::cout<<"C++ detected model modification"<<std::endl;
      loadModel(viewer, V, F, NF, model);
      last_modified_time = utils->last_modified_time("./data/timestamp");
      model->paused = false;

    }

    viewer.data().clear_labels();

    model->camera_rad += model->rad_per_frame;
    viewer.core().camera_eye.z() = sin(model->camera_rad) * model->camera_radius;
    viewer.core().camera_eye.x() = cos(model->camera_rad) * model->camera_radius;

    model->step(model->steps_per_frame);
    viewer.data().set_mesh(model->V,model->F);
    viewer.data().compute_normals();
    return true;
  };


  // menu
  menu.callback_draw_viewer_window = [&]()
  {
    // remove the default window
  };

  menu.callback_draw_custom_window =[&]()
  {
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    ImGui::SetNextWindowPos(ImVec2(1.0 * mode->width / 2 - 150, 0), ImGuiSetCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(150, 0), ImGuiSetCond_FirstUseEver);

    // archived load function
    {
//    if (ImGui::Button("Load"))
//    {
//      std::string input_dir = igl::file_dialog_open();
//      string root_dir = ROOT_DIR;
//      root_dir += '/';
//      Pre processor p = Preprocessor(input_dir, root_dir);
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
    }

    ImGui::Begin(
      "Setting", nullptr,
      ImGuiWindowFlags_NoSavedSettings
    );

    ImGui::InputDouble("tensile factor", &model->k_s, 0.001, 0.01, "%.3g");
    ImGui::InputDouble("dielectric factor", &model->k_e, 0.001, 0.01, "%.3g");
    ImGui::InputDouble("bending factor", &model->k_b, 0.001, 0.01, "%.3g");
    ImGui::InputDouble("damping", &model->damping, 0.001, 0.01, "%.3g");
    ImGui::InputDouble("damping factor", &model->damping_coeff, 0.001, 0.01, "%.3g");
    ImGui::InputDouble("step size", &model->h);
    ImGui::InputInt("steps per frame", &model->steps_per_frame);
    ImGui::InputFloat("rad per frame", &model->rad_per_frame, 0.01, 0.1, "%.2g");

    if (ImGui::Button("begin")) {
      model->paused = false;
    }

    if (ImGui::Button("paused")) {
      model->paused = true;
    }

    if (ImGui::Button("Reset")) {
      loadModel(viewer, V, F, NF, model);
    }

    if (ImGui::Button("Download")) {
      char buff[100];
      sprintf(buff, "%s/data/download/output.obj", ROOT_DIR);
      string output_dir = buff;
      igl::writeOBJ(output_dir, model->V, model->F);
    }

    static int num_choices = 0;
//    if (ImGui::InputInt("Num letters", &num_choices))
//    {
//      model->show_bending_force_i = false;
//      model->show_bending_force_j = false;
//      model->show_bending_force_k = false;
//      model->show_bending_force_l = false;
//      model->show_edge_force = false;
//      model->show_electrostatic_force = false;
//
//
//      switch(num_choices) {
//
//        case 0 :
//          model->show_bending_force_i = true;
//          break;
//
//        case 1 :
//          model->show_bending_force_j = true;
//          break;
//
//        case 2 :
//          model->show_bending_force_k = true;
//          break;
//
//        case 3 :
//          model->show_bending_force_l = true;
//          break;
//
//        case 4 :
//          model->show_electrostatic_force = true;
//          break;
//
//        case 5 :
//          model->show_edge_force = true;
//          break;
//
//        default :
//          cout<<"default"<<endl;
//          break;
//
//      }
//
//    }
    ImGui::End();
  };

  // viewer settings
  viewer.callback_init = [&](igl::opengl::glfw::Viewer& viewer) -> bool {
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    glfwSetWindowPos(viewer.window, mode->width / 2, 0);
    glfwSetWindowSize(viewer.window, mode->width / 2, mode->height*2);
    return false;
  };

  viewer.plugins.push_back(&menu);
  viewer.core().is_animating = true;
  viewer.core().background_color = Eigen::Vector4f(0.9, 0.9, 0.9, 1.0);

  viewer.launch();
}
