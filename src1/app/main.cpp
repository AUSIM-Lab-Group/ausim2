#include <aimrte/program/app_mode.h>

#include "aima/sim/module/core/core.h"

/**
 * @brief 定义了 main 函数，并注册了两个同类型模块。
 *        源码编译、并启动本程序时，先执行配置部署目标：example-program-app_mode_install-SETUP_ALL
 *        再进入本程序所在目录，带着参数
 *          --cfg_file_path=cfg/app_mode_config.yaml
 *        执行本程序。
 */

// AIMRTE_APP_MAIN(("MotionControlModule", mc::module::MotionControlModule()))

int main(int argc, char** argv) {
  aimrte::Cfg cfg(argc, argv, "sim");

  // 给定要加载的模块（可以给定多个），并启动框架
  return aimrte::Run(cfg, {{"MujocoSimModule", std::make_shared<sim::module::MujocoSimModule>()}});
}
