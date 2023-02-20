// Copyright 2022-2023, Moshi Turner
// Copyright 2022, Daniel Willmott
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Utilities to find paths of things.
 * @author Moshi Turner <mosesturner@protonmail.com>
 * @author Daniel Willmott <web@dan-w.com>
 */

#pragma once

#include <string>

/**
 * gets the absolute path to the root directory of the driver
 */
bool GetDriverRootPath(std::string& out_path);

bool GetHMDConfig(std::string &out_path, bool pull = true);

bool GetHandTrackingModelsPath(std::string &out_path);