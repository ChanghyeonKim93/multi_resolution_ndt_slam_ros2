/*
Copyright 2025 Changhyeon Kim
e-mail: hyun91015@gmail.com
*/

#ifndef UTILITY_TIME_MONITOR_H_
#define UTILITY_TIME_MONITOR_H_

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#define CHECK_ELAPSED_TIME_OF_THIS_FUNCTION_FROM_THIS(activate)                \
  std::unique_ptr<utility::TimeMonitor> _time_monitor_ =                       \
      ((activate) ? std::make_unique<utility::TimeMonitor>(__FILE__, __func__) \
                  : nullptr);

namespace utility {

struct Timestamp {
  int64_t sec{0};
  int32_t nsec{0};
};

using StdClock = std::chrono::system_clock;
using StdTimePoint = StdClock::time_point;

inline StdTimePoint GetCurrentTimePoint() { return StdClock::now(); }

inline Timestamp ConvertToTimestamp(const StdTimePoint& time_point) {
  static constexpr int64_t kOneSecInNanosec =
      static_cast<int64_t>(1'000'000'000);
  Timestamp timestamp;
  timestamp.sec = static_cast<int64_t>(time_point.time_since_epoch().count()) /
                  kOneSecInNanosec;
  timestamp.nsec = static_cast<int32_t>(time_point.time_since_epoch().count() -
                                        timestamp.sec * kOneSecInNanosec);
  return timestamp;
}

/// @brief Class for monitoring execution time of function
class TimeMonitor {
 public:
  /// @brief Constructor of time monitor
  /// @param file_name File name where the function exists
  /// @param function_name Function name
  TimeMonitor(const std::string& file_name, const std::string& function_name);

  /// @brief Destructor of time monitor
  ~TimeMonitor();

 private:
  const std::string name_;
  const StdTimePoint start_time_point_;
};

/// @brief Class for calculating and showing time statistics of time monitors
class TimeMonitorManager {
  friend class TimeMonitor;

 public:
  TimeMonitorManager(const TimeMonitorManager& time_monitor_manager) = delete;

  /// @brief Get singleton instance of time monitor manager
  /// @return Pointer of singleton instance of time monitor manager
  static TimeMonitorManager* GetSingletonObjectPointer();

 protected:
  void RegisterFunctionTime(const std::string& name,
                            const Timestamp& start_timestamp,
                            const int64_t elapsed_time_in_nsec);

 private:
  TimeMonitorManager();
  ~TimeMonitorManager();
  void SortTimeList(std::vector<std::pair<Timestamp, int64_t>>* time_list);
  int64_t ComputeSum(
      const std::vector<std::pair<Timestamp, int64_t>>& time_list);
  int64_t ComputeMedian(
      const std::vector<std::pair<Timestamp, int64_t>>& time_list);
  int64_t ComputeMean(
      const std::vector<std::pair<Timestamp, int64_t>>& time_list);
  int64_t ComputeStandardDevication(
      const std::vector<std::pair<Timestamp, int64_t>>& time_list);

  std::mutex mutex_;
  std::map<std::string, std::vector<std::pair<Timestamp, int64_t>>>
      time_list_map_;
};

}  // namespace utility

#endif  // UTILITY_TIME_MONITOR_H_
