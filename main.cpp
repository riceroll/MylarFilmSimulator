#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <thread>
#include <string>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split

#include <imgui/imgui.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/unproject_ray.h>
#include <igl/heat_geodesics.h>
//#include <igl/upsample.h>
//#include <igl/colormap.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

// Eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/Dense>


#include "Model.h"

using namespace std;


int main(int argc, char **argv) {


  igl::opengl::glfw::Viewer viewer;
  viewer.core().is_animating = true;

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::readOBJ("/Users/Roll/Desktop/test.obj", V, F);

  Model model(V, F);

  viewer.data().set_mesh(model.V,model.F);



  viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer& viewer)->bool {
    model.step();
    viewer.data().set_mesh(model.V,model.F);
    return true;
  };

  viewer.launch();
}
