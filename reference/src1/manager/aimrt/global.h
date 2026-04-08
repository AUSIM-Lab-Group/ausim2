#pragma once

#include "aimrt_module_cpp_interface/logger/logger.h"

void SetLogger(aimrt::logger::LoggerRef);
aimrt::logger::LoggerRef GetLogger();
