#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace auto_sound::test {

inline std::vector<std::vector<double>> ReadCsvDoubles(const std::string& path,
                                                       bool skip_header = true) {
  std::vector<std::vector<double>> rows;
  std::ifstream file(path);
  if (!file) {
    return rows;
  }

  std::string line;
  bool first = true;
  while (std::getline(file, line)) {
    if (first && skip_header) {
      first = false;
      continue;
    }
    first = false;
    if (line.empty()) {
      continue;
    }

    std::vector<double> values;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      if (cell.empty()) {
        values.push_back(0.0);
      } else {
        values.push_back(std::stod(cell));
      }
    }
    rows.push_back(values);
  }

  return rows;
}

inline void WriteLines(const std::string& path, const std::vector<std::string>& lines) {
  std::ofstream file(path, std::ios::trunc);
  for (const auto& line : lines) {
    file << line << '\n';
  }
}

}  // namespace auto_sound::test
