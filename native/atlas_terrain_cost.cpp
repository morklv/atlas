// ATLAS native terrain feasibility core.
// Input: whitespace-separated elevation rows; use `nan` for unknown cells.
// Output: a JSON report and a row-major 0/1 traversability mask.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

struct Options {
  double resolution_m = 0.5;
  double max_step_m = 0.25;
  double max_slope_deg = 20.0;
};

bool parse_double(const std::string& text, double* value) {
  if (text == "nan" || text == "NaN" || text == "NAN") {
    *value = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  char* end = nullptr;
  *value = std::strtod(text.c_str(), &end);
  return end != text.c_str() && *end == '\0';
}

int main(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    std::string flag = argv[i];
    if (flag == "--resolution" && i + 1 < argc) options.resolution_m = std::atof(argv[++i]);
    else if (flag == "--max-step" && i + 1 < argc) options.max_step_m = std::atof(argv[++i]);
    else if (flag == "--max-slope" && i + 1 < argc) options.max_slope_deg = std::atof(argv[++i]);
    else {
      std::cerr << "Usage: atlas_terrain_cost [--resolution metres] [--max-step metres] [--max-slope degrees] < elevation.txt\n";
      return 2;
    }
  }
  if (options.resolution_m <= 0 || options.max_step_m < 0 || options.max_slope_deg < 0) {
    std::cerr << "All limits must be non-negative and resolution must be positive.\n";
    return 2;
  }

  std::vector<std::vector<double>> elevation;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream stream(line);
    std::string token;
    std::vector<double> row;
    while (stream >> token) {
      double value;
      if (!parse_double(token, &value)) {
        std::cerr << "Invalid elevation value: " << token << "\n";
        return 2;
      }
      row.push_back(value);
    }
    if (!row.empty()) elevation.push_back(row);
  }
  if (elevation.empty()) { std::cerr << "No elevation rows received.\n"; return 2; }
  const size_t width = elevation.front().size();
  if (width == 0) { std::cerr << "Empty elevation row.\n"; return 2; }
  for (const auto& row : elevation) {
    if (row.size() != width) { std::cerr << "Elevation rows must have equal width.\n"; return 2; }
  }

  const size_t height = elevation.size();
  std::vector<int> traversable(width * height, 0);
  size_t observed = 0, blocked_step = 0, blocked_slope = 0, safe = 0;
  const double max_slope_radians = options.max_slope_deg * std::acos(-1.0) / 180.0;
  const int dx[] = {-1, 1, 0, 0};
  const int dy[] = {0, 0, -1, 1};
  for (size_t y = 0; y < height; ++y) for (size_t x = 0; x < width; ++x) {
    double current = elevation[y][x];
    if (!std::isfinite(current)) continue;
    ++observed;
    bool has_neighbour = false, step_fail = false, slope_fail = false;
    for (int n = 0; n < 4; ++n) {
      int nx = static_cast<int>(x) + dx[n], ny = static_cast<int>(y) + dy[n];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) || ny >= static_cast<int>(height)) continue;
      double neighbour = elevation[ny][nx];
      if (!std::isfinite(neighbour)) continue;
      has_neighbour = true;
      const double rise = std::abs(current - neighbour);
      if (rise > options.max_step_m) step_fail = true;
      if (std::atan(rise / options.resolution_m) > max_slope_radians) slope_fail = true;
    }
    if (!has_neighbour || step_fail || slope_fail) {
      if (step_fail) ++blocked_step;
      if (slope_fail) ++blocked_slope;
      continue;
    }
    traversable[y * width + x] = 1;
    ++safe;
  }

  std::cout << std::setprecision(10);
  std::cout << "{\n  \"schema\": \"atlas-native-terrain-cost-v1\",\n";
  std::cout << "  \"width\": " << width << ",\n  \"height\": " << height << ",\n";
  std::cout << "  \"resolution_m\": " << options.resolution_m << ",\n";
  std::cout << "  \"limits\": {\"max_step_m\": " << options.max_step_m << ", \"max_slope_deg\": " << options.max_slope_deg << "},\n";
  std::cout << "  \"metrics\": {\"observed_cells\": " << observed << ", \"traversable_cells\": " << safe << ", \"blocked_by_step\": " << blocked_step << ", \"blocked_by_slope\": " << blocked_slope << "},\n";
  std::cout << "  \"traversable_mask\": [";
  for (size_t i = 0; i < traversable.size(); ++i) { if (i) std::cout << ", "; std::cout << traversable[i]; }
  std::cout << "]\n}\n";
}
