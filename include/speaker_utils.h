#pragma once

#include <cstdint>
#include <string>
#include <vector>

std::vector<std::string> split_command(const std::string& command);
void clear_screen();
void prompt_enter_to_continue();

bool try_stoul_strict(const std::string& str, uint32_t& out);

// Camera network params display helper functions
std::string bitrate_to_string(int bitrate);
std::string resolution_to_string(int resolution);
std::string encoding_to_string(int encoding);

int string_to_bitrate(const std::string& str);
int string_to_resolution(const std::string& str);
int string_to_encoding(const std::string& str);
