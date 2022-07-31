#include <filesystem>
#include <regex>
#include <fstream>

#include "vr_utils.h"
#include "win_utils.h"
#include "driver_log.h"

bool GetDriverRootPath(std::string &out_path) {
    std::string dll_path;
    if (!GetDLLPath(dll_path)) return false;

    const std::string unwanted = R"(\bin\win64\)";
    out_path = dll_path.substr(0, dll_path.find_last_of("\\/")).erase(dll_path.find(unwanted), unwanted.length());

    return true;
}

static bool GetDriverResourceInternalPath(std::string &out_path) {
    std::string root_path;
    if (!GetDriverRootPath(root_path)) return false;

    //yeet
    out_path = root_path + "\\resources\\internal";

    return true;
}

static bool FindHMDConfig(const std::string &search_path, std::string &out_config) {
    DriverLog("Searching for HMD configs in path: %s", search_path.c_str());

    for (const auto &file: std::filesystem::directory_iterator(search_path)) {
        std::filesystem::path file_path(file);
        const std::string file_name = file_path.filename().string();

        DriverLog("Current file: %s", file_name.c_str());

        if (std::regex_match(file_name, std::regex("LHR-[A-Z0-9]*\\.json"))) {
            DriverLog("Using HMD config from: %s", file_name.c_str());

            std::ifstream ifs(file_path);
            std::string content((std::istreambuf_iterator<char>(ifs)), (std::istreambuf_iterator<char>()));

            out_config = content;

            return true;
        }
    }

    return false;
}

/***
 * Creates a HMD config. Assume
 * @param resources_path
 * @return
 */
static bool CreateHMDConfig(const std::string &internal_resources_path) {
    if (!CreateProcessWaitTerminate(internal_resources_path, "lighthouse_console.exe", "downloadconfig", 10000))
        return false;

    return true;
}

bool GetHMDConfig(std::string &out_path, bool pull) {
    std::string res_path;
    if (!GetDriverResourceInternalPath(res_path)) return false;

    if (!FindHMDConfig(res_path, out_path)) {
        if (!pull) {
            DriverLog("Failed to get a HMD config.");
            return false;
        }
        DriverLog("HMD config was not found. Attempting to pull config...");

        if (!CreateHMDConfig(res_path)) {
            DriverLog("Failed to pull config");
            return false;
        }

        return GetHMDConfig(out_path, false);
    }

    return true;
}
