/*
Copyright 2025 Changhyeon Kim
e-mail: hyun91015@gmail.com
*/

#include "utility/time_monitor.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace utility {

TimeMonitorManager* TimeMonitorManager::GetSingletonObjectPointer() {
  static TimeMonitorManager time_monitor_manager;
  return &time_monitor_manager;
}

TimeMonitor::TimeMonitor(const std::string& file_name,
                         const std::string& function_name)
    : name_(file_name + "/" + function_name),
      start_time_point_(GetCurrentTimePoint()) {}

TimeMonitor::~TimeMonitor() {
  const int64_t elapsed_time_in_nsec =
      static_cast<int64_t>((GetCurrentTimePoint() - start_time_point_).count());
  TimeMonitorManager::GetSingletonObjectPointer()->RegisterFunctionTime(
      name_, ConvertToTimestamp(start_time_point_), elapsed_time_in_nsec);
}

TimeMonitorManager::TimeMonitorManager() {}

void TimeMonitorManager::RegisterFunctionTime(
    const std::string& name, const Timestamp& start_timestamp,
    const int64_t elapsed_time_in_nsec) {
  mutex_.lock();
  if (time_list_map_.find(name) == time_list_map_.end())
    time_list_map_.insert({name, std::vector<std::pair<Timestamp, int64_t>>()});
  time_list_map_.at(name).push_back({start_timestamp, elapsed_time_in_nsec});
  mutex_.unlock();
}

TimeMonitorManager::~TimeMonitorManager() {
  struct Statistics {
    std::string function_name{""};
    int64_t calls{0};
    int64_t min{0};
    int64_t max{0};
    int64_t avg{0};
    int64_t med{0};
    int64_t std{0};
    int64_t sum{0};
  };

  std::vector<Statistics> statistics_list;

  std::lock_guard<std::mutex> local_lock(mutex_);
  for (auto& [name, time_list] : time_list_map_) {
    SortTimeList(&time_list);
    if (time_list.empty()) continue;
    Statistics statistics{name,
                          static_cast<int>(time_list.size()),
                          time_list.front().second,
                          time_list.back().second,
                          ComputeMean(time_list),
                          ComputeMedian(time_list),
                          ComputeStandardDevication(time_list),
                          ComputeSum(time_list)};
    statistics_list.push_back(statistics);
  }

  std::sort(statistics_list.begin(), statistics_list.end(),
            [](const Statistics& lhs, const Statistics& rhs) {
              return lhs.sum > rhs.sum;
            });
  std::stringstream ss;
  ss << "============== Function Time Statistics ==============" << std::endl;
  ss << "Time is sorted by \"total time\" of the function.\n";
  ss.precision(6);
  ss.setf(std::ios::fixed);
  for (auto& statistics : statistics_list) {
    ss << statistics.function_name << "\n";
    ss << "# Calls   : " << statistics.calls << "\n";
    ss << "Time Min. : " << statistics.min * 1e-6 << " [ms]\n";
    ss << "Time Max. : " << statistics.max * 1e-6 << " [ms]\n";
    ss << "Time Avg. : " << statistics.avg * 1e-6 << " [ms]\n";
    ss << "Time Med. : " << statistics.med * 1e-6 << " [ms]\n";
    ss << "Time Std. : " << statistics.std * 1e-6 << " [ms]\n";
    ss << "Time Total: " << statistics.sum * 1e-6 << " [ms]\n";
    ss << "___________________________________________" << std::endl;
  }
  std::cerr << ss.str();
}

void TimeMonitorManager::SortTimeList(
    std::vector<std::pair<Timestamp, int64_t>>* time_list) {
  std::sort(time_list->begin(), time_list->end(),
            [](const std::pair<Timestamp, int64_t>& lhs,
               const std::pair<Timestamp, int64_t>& rhs) {
              return lhs.second < rhs.second;
            });
}

int64_t TimeMonitorManager::ComputeSum(
    const std::vector<std::pair<Timestamp, int64_t>>& time_list) {
  int64_t time_sum = 0;
  for (const auto& [timestamp, elapsed_time] : time_list)
    time_sum += elapsed_time;
  return time_sum;
}

int64_t TimeMonitorManager::ComputeMedian(
    const std::vector<std::pair<Timestamp, int64_t>>& time_list) {
  return time_list.at(time_list.size() / 2).second;
}

int64_t TimeMonitorManager::ComputeMean(
    const std::vector<std::pair<Timestamp, int64_t>>& time_list) {
  if (time_list.empty()) return 0.0;
  const int64_t num_data = static_cast<int64_t>(time_list.size());
  return (ComputeSum(time_list) / num_data);
}

int64_t TimeMonitorManager::ComputeStandardDevication(
    const std::vector<std::pair<Timestamp, int64_t>>& time_list) {
  if (time_list.size() <= 1) return 0.0;
  int64_t squared_time_sum = 0.0;
  for (const auto& [timestamp, elapsed_time] : time_list)
    squared_time_sum += elapsed_time * elapsed_time;
  const auto mean_time = ComputeMean(time_list);
  const auto num_data = static_cast<int64_t>(time_list.size());
  return static_cast<int64_t>(
      std::sqrt(squared_time_sum / num_data - mean_time * mean_time));
}

}  // namespace utility
