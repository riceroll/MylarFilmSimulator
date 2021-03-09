//
// Created by 莱洛 on 2021/2/26.
//
#include "Preprocessor.h"
//#include <svgpp/policy/xml/rapidxml_ns.hpp>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>


using namespace std;

std::string exec(const char* cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

Preprocessor::Preprocessor(string file_dir, string root_dir) {

  string python_dir = root_dir + "preprocess.py";
  string command = "run " + python_dir + " " + file_dir + " " + root_dir;
  cout<<command<<endl;
  char buff[100];
  strcpy(buff, command.c_str());
//  string result = exec(buff);

  std::system(command.c_str());
}
