#pragma once

#include <string>

bool GetDLLPath(std::string& out_string);

/**
 * Creates a process and waits for it to terminate
 */
bool CreateProcessWaitTerminate(const std::string &executable_path, const std::string& executable_name, const std::string &arguments, int timeout);

std::string GetLastErrorAsString();