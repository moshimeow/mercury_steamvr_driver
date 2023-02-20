#pragma once

#include <string>

/**
 * gets the absolute path to the root directory of the driver
 */
bool GetDriverRootPath(std::string& out_path);

bool GetHMDConfig(std::string &out_path, bool pull = true);

// bool GetHandTrackingModelsPath(std::);