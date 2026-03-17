#include "speaker_utils.h"

#include <iostream>
#include <sstream>

std::vector<std::string> split_command(const std::string& command) {
  std::vector<std::string> args;
  std::istringstream iss(command);
  std::string token;
  while (iss >> token) {
    args.push_back(token);
  }
  return args;
}

void clear_screen() { std::cout << "\033[2J\033[1;1H"; }

void prompt_enter_to_continue() {
  std::cout << "Press Enter to continue...";
  std::cin.get();
}

bool try_stoul_strict(const std::string& str, uint32_t& out) {
  try {
    size_t pos = 0;
    out = std::stoul(str, &pos);
    return pos == str.size();
  } catch (...) {
    return false;
  }
}

// Camera network params display helper functions
std::string bitrate_to_string(int bitrate) {
  switch (bitrate) {
    case 0:
      return "Default";
    case 1:
      return "0.5M";
    case 2:
      return "1M";
    case 3:
      return "2M";
    case 4:
      return "3M";
    case 5:
      return "4M";
    case 6:
      return "0.1M";
    case 7:
      return "0.2M";
    default:
      return "Unknown";
  }
}

std::string resolution_to_string(int resolution) {
  switch (resolution) {
    case 0:
      return "Default";
    case 1:
      return "1080p";
    case 2:
      return "720p";
    case 3:
      return "576p";
    default:
      return "Unknown";
  }
}

std::string encoding_to_string(int encoding) {
  switch (encoding) {
    case 0:
      return "Default";
    case 1:
      return "H264";
    case 2:
      return "H265";
    default:
      return "Unknown";
  }
}

int string_to_bitrate(const std::string& str) {
  if (str == "0.5M") return 1;
  if (str == "1M") return 2;
  if (str == "2M") return 3;
  if (str == "3M") return 4;
  if (str == "4M") return 5;
  if (str == "0.1M") return 6;
  if (str == "0.2M") return 7;
  return 0;  // Default
}

int string_to_resolution(const std::string& str) {
  if (str == "1080" || str == "1080p") return 1;
  if (str == "720" || str == "720p") return 2;
  if (str == "576" || str == "576p") return 3;
  return 0;  // Default
}

int string_to_encoding(const std::string& str) {
  if (str == "H264" || str == "h264") return 1;
  if (str == "H265" || str == "h265") return 2;
  return 0;  // Default
}
