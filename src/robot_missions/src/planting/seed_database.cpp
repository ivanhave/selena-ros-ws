#include "robot_missions/planting/seed_database.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace robot_missions
{

static rclcpp::Logger logger() { return rclcpp::get_logger("SeedDatabase"); }

bool SeedDatabase::load(const std::string & csv_path)
{
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger(), "Failed to open: %s", csv_path.c_str());
        return false;
    }

    profiles_.clear();
    std::string line;

    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#') continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            tokens.push_back(token);
        }

        if (tokens[0] == "name") continue;

        if (tokens.size() < 6) {
            RCLCPP_WARN(logger(), "Skipping malformed line: %s", line.c_str());
            continue;
        }

        SeedProfile p;
        try {
            p.name          = tokens[0];
            p.spacing_x_mm  = std::stof(tokens[1]);
            p.spacing_y_mm  = std::stof(tokens[2]);
            p.depth_mm      = std::stof(tokens[3]);
            p.tool_id       = std::stoul(tokens[4]);
            p.tool_param    = std::stoul(tokens[5]);
        } catch (const std::exception & e) {
            RCLCPP_WARN(logger(), "Skipping malformed line (parse error: %s): %s",
                e.what(), line.c_str());
            continue;
        }

        profiles_.push_back(p);
        RCLCPP_INFO(logger(),
            "Loaded: %s sx=%.1f sy=%.1f depth=%.1f tool=%u tool_param=%u",
            p.name.c_str(), p.spacing_x_mm, p.spacing_y_mm,
            p.depth_mm, p.tool_id, p.tool_param);
    }

    RCLCPP_INFO(logger(), "Loaded %zu seed profiles.", profiles_.size());
    return !profiles_.empty();
}

bool SeedDatabase::get_profile(
    const std::string & name,
    SeedProfile & profile) const
{
    for (const auto & p : profiles_) {
        if (p.name == name) {
            profile = p;
            return true;
        }
    }
    RCLCPP_ERROR(logger(), "Seed not found: %s", name.c_str());
    return false;
}

std::vector<std::string> SeedDatabase::get_all_names() const
{
    std::vector<std::string> names;
    for (const auto & p : profiles_) {
        names.push_back(p.name);
    }
    return names;
}

} // namespace robot_missions
