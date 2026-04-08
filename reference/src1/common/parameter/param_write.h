#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "./param_util.h"

namespace param {

template <sequence_container T>
bool WriteParam(YAML::Node& node, const T& p) {
  return ToYaml(node, p);
}

template <refletable T>
bool WriteParam(YAML::Node& node, const T& p) {
  return ToYaml(node, p);
}

template <class T>
bool WriteParam(YAML::Node& pn, const T& param, const std::string param_name, const std::string delimiter) {
  // 将参数写到node中，如果出现重复的key，会覆盖
  YAML::Node node;
  bool ret = false;
  try {
    ret = WriteParam(node, param);

    if (!ret) {
      return false;
    }

    std::vector<std::string> tokens;
    SplitString(tokens, param_name, delimiter);

    // 倒序遍历tokens，构造一颗树，将node写入到树中
    for (auto iter = tokens.rbegin(); iter != tokens.rend(); ++iter) {
      YAML::Node temp;
      temp[*iter] = node;
      node = temp;
    }

    pn = node;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

template <class T>
bool WriteParam(const T& param, const std::string write_yaml_file, const std::string param_name = "", const std::string delimiter = ".") {
  YAML::Node node;
  bool ret = false;
  try {
    ret = WriteParam(node, param, param_name, delimiter);
    if (!ret) {
      return false;
    }

    // 判断文件是否存在
    if (std::filesystem::exists(write_yaml_file)) {
      // 如果文件存在，读取文件内容
      YAML::Node old_node = YAML::LoadFile(write_yaml_file);
      // 将node写入到old_node中
      ret = Merge(old_node, node);
      if (!ret) {
        return false;
      }
      node = old_node;
    }

    // 将node写入到文件中
    std::ofstream fout(write_yaml_file, std::ios::trunc);
    fout << node;
    fout.close();

  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

template <class T>
bool WriteParamToNode(YAML::Node& out_node, const T& param, const std::string param_name = "", const std::string delimiter = ".") {
  YAML::Node node;
  YAML::Node old_node = out_node;
  bool ret = false;
  try {
    ret = WriteParam(node, param, param_name, delimiter);
    if (!ret) {
      return false;
    }

    // 将node写入到old_node中
    ret = Merge(old_node, node);
    if (!ret) {
      return false;
    }
    out_node = old_node;
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

template <class T>
bool WriteParamToString(std::string& out_str, const T& param, const std::string param_name = "", const std::string delimiter = ".") {
  YAML::Node node;
  bool ret = false;
  try {
    ret = WriteParam(node, param, param_name, delimiter);
    if (!ret) {
      return false;
    }
    // 将node写入到old_node中
    out_str = YAML::Dump(node);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

}  // namespace param
