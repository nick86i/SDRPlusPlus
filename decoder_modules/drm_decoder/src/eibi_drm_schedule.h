#pragma once
#include <ctime>
#include <string>
#include <vector>
struct DrmScheduleEntry { double frequencyKHz=0; std::string time,days,station,period,startDate,stopDate; };
std::vector<DrmScheduleEntry> loadDrmSchedule(const std::string& path);
bool drmScheduleActiveNow(const DrmScheduleEntry&, const std::tm& utc);
std::string drmScheduleLocalTime(const DrmScheduleEntry&);
std::string drmScheduleFrequency(double frequencyKHz);
