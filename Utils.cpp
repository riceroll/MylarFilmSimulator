//
// Created by 莱洛 on 2021/3/29.
//

#include "Utils.h"
#include <iostream>

Utils::Utils() {

}

unsigned int Utils::last_modified_time(std::string filename) {
  std::ifstream ifile(filename);
  std::string line;
  std::getline(ifile, line);
  unsigned int time = std::stoi(line);
  return time;
}
