#ifndef HEXAPOD_HARDWARE_INTERFACE__HARDWARE_TESTS__TEST_COMMON_HPP_
#define HEXAPOD_HARDWARE_INTERFACE__HARDWARE_TESTS__TEST_COMMON_HPP_

#include <cmath>
#include <csignal>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace hardware_tests {

inline volatile std::sig_atomic_t stop_requested = 0;

/**
 * @brief Make Ctrl+C set stop_requested instead of killing the program, so tests can finish cleanly
*/
inline void catchCtrlC() {
    std::signal(SIGINT, [](int) { stop_requested = 1; });
}

/**
 * @brief Command line arguments: "--name value" options, "--name" flags and positional arguments
*/
class Args {
public:
    Args(int argc, char** argv, const std::vector<std::string>& flag_names = {}) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.rfind("--", 0) != 0) {
                positional_.push_back(arg);
                continue;
            }
            std::string name = arg.substr(2);
            bool is_flag = name == "help";
            for (const auto& flag : flag_names) {
                is_flag = is_flag || name == flag;
            }
            if (is_flag || i + 1 >= argc) {
                options_[name] = "";
            } else {
                options_[name] = argv[++i];
            }
        }
    }

    bool has(const std::string& name) const { return options_.count(name) > 0; }
    const std::vector<std::string>& positional() const { return positional_; }

    std::string get(const std::string& name, const std::string& fallback) const {
        auto it = options_.find(name);
        return it != options_.end() ? it->second : fallback;
    }
    // Accepts hex, e.g. --address 0x6A
    int getInt(const std::string& name, int fallback) const {
        return has(name) ? std::stoi(options_.at(name), nullptr, 0) : fallback;
    }
    double getDouble(const std::string& name, double fallback) const {
        return has(name) ? std::stod(options_.at(name)) : fallback;
    }

private:
    std::map<std::string, std::string> options_;
    std::vector<std::string> positional_;
};

/**
 * @brief Running mean and standard deviation of a value
*/
class Stats {
public:
    void add(double value) {
        count_++;
        sum_ += value;
        sum_sq_ += value * value;
        min_ = count_ == 1 || value < min_ ? value : min_;
        max_ = count_ == 1 || value > max_ ? value : max_;
    }
    int count() const { return count_; }
    double mean() const { return count_ ? sum_ / count_ : 0.0; }
    double stddev() const {
        if (count_ < 2) {
            return 0.0;
        }
        double variance = (sum_sq_ - sum_ * sum_ / count_) / (count_ - 1);
        return variance > 0.0 ? std::sqrt(variance) : 0.0;
    }
    double min() const { return min_; }
    double max() const { return max_; }

private:
    int count_ = 0;
    double sum_ = 0.0;
    double sum_sq_ = 0.0;
    double min_ = 0.0;
    double max_ = 0.0;
};

/**
 * @brief Counts PASS/WARN/FAIL results and prints each one
*/
class Report {
public:
    void pass(const std::string& text) { printf("  PASS  %s\n", text.c_str()); }
    void warn(const std::string& text) { warnings_++; printf("  WARN  %s\n", text.c_str()); }
    void fail(const std::string& text) { failures_++; printf("  FAIL  %s\n", text.c_str()); }
    void info(const std::string& text) { printf("        %s\n", text.c_str()); }

    /**
     * @brief Print the overall result
     * @return The process exit code: 0 unless something failed
    */
    int finish() const {
        printf("\nResult: %s (%d warning%s, %d failure%s)\n",
            failures_ ? "FAIL" : (warnings_ ? "PASS with warnings" : "PASS"),
            warnings_, warnings_ == 1 ? "" : "s", failures_, failures_ == 1 ? "" : "s");
        return failures_ ? 1 : 0;
    }

private:
    int warnings_ = 0;
    int failures_ = 0;
};

/**
 * @brief printf into a std::string
*/
template <typename... T>
std::string format(const char* fmt, T... values) {
    std::string text(static_cast<size_t>(snprintf(nullptr, 0, fmt, values...)), '\0');
    snprintf(text.data(), text.size() + 1, fmt, values...);
    return text;
}

}  // namespace hardware_tests

#endif  // HEXAPOD_HARDWARE_INTERFACE__HARDWARE_TESTS__TEST_COMMON_HPP_
