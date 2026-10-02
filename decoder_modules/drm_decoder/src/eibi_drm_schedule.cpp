#include "eibi_drm_schedule.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
std::vector<std::string> fields(const std::string& line) { std::vector<std::string> out; std::string v; bool q=false; for(char c:line){if(c=='"')q=!q;else if(c==';'&&!q){out.push_back(v);v.clear();}else v+=c;} out.push_back(v); return out; }
int mins(const std::string& s){try{return std::stoi(s.substr(0,2))*60+std::stoi(s.substr(2,2));}catch(...){return -1;}}
int dayCode(const std::string& s){static const char* n[]={"Mo","Tu","We","Th","Fr","Sa","Su"};for(int i=0;i<7;i++)if(s==n[i])return i+1;return 0;}
bool dayPart(const std::string& s,int d){auto p=s.find('-');if(p!=std::string::npos){int a=dayCode(s.substr(0,p)),b=dayCode(s.substr(p+1));if(a&&b)return a<=b?(d>=a&&d<=b):(d>=a||d<=b);}for(size_t i=0;i+1<s.size();i+=2)if(dayCode(s.substr(i,2))==d)return true;return dayCode(s)==d;}
bool dayOk(const std::string& s,int d,int md,int mo){if(s.empty())return true;if(std::all_of(s.begin(),s.end(),[](unsigned char c){return std::isdigit(c);}))return s.find(char('0'+d))!=std::string::npos;static const char* m[]={"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};for(int i=0;i<12;i++){auto p=s.find(m[i]);if(p!=std::string::npos){try{return std::stoi(s.substr(0,p))==md&&mo==i+1;}catch(...){return false;}}}if(s.rfind("MF",0)==0)return d<=5;if(s.find("irr")!=std::string::npos||s.find("test")!=std::string::npos||s.find("tent")!=std::string::npos)return true;std::istringstream in(s);std::string x;while(std::getline(in,x,','))if(dayPart(x,d))return true;return false;}
bool dateOk(const DrmScheduleEntry&e,int d,int m){if(e.period!="6"||e.startDate.size()<4||e.stopDate.size()<4)return true;auto p=[](const std::string&s){try{return std::stoi(s.substr(2,2))*100+std::stoi(s.substr(0,2));}catch(...){return -1;}};int n=m*100+d,a=p(e.startDate),b=p(e.stopDate);return a<0||b<0?true:(a<=b?(n>=a&&n<=b):(n>=a||n<=b));}
}
std::vector<DrmScheduleEntry> loadDrmSchedule(const std::string& path){std::ifstream f(path,std::ios::binary);std::vector<DrmScheduleEntry> out;std::string l;std::getline(f,l);while(std::getline(f,l)){auto x=fields(l);if(x.size()<8||x[4].find("DIGITAL")==std::string::npos)continue;try{DrmScheduleEntry e;e.frequencyKHz=std::stod(x[0]);if(e.frequencyKHz<2300||e.frequencyKHz>30000)continue;e.time=x[1];e.days=x[2];e.station=x[4];e.period=x.size()>8?x[8]:"";e.startDate=x.size()>9?x[9]:"";e.stopDate=x.size()>10?x[10]:"";auto p=e.station.rfind(" DIGITAL");if(p!=std::string::npos)e.station.erase(p);out.push_back(std::move(e));}catch(...){}}return out;}
bool drmScheduleActiveNow(const DrmScheduleEntry&e,const std::tm&u){if(e.time.size()<9||e.time[4]!='-')return false;int a=mins(e.time),b=mins(e.time.substr(5)),n=u.tm_hour*60+u.tm_min;if(a<0||b<0)return false;bool wrap=b<=a&&b!=1440;if(!(wrap?(n>=a||n<b):(n>=a&&n<b)))return false;int d=u.tm_wday?u.tm_wday:7;if(wrap&&n<b)d=d==1?7:d-1;return dayOk(e.days,d,u.tm_mday,u.tm_mon+1)&&dateOk(e,u.tm_mday,u.tm_mon+1);}
std::string drmScheduleLocalTime(const DrmScheduleEntry&e){int a=mins(e.time),b=mins(e.time.substr(5));if(a<0||b<0)return e.time;if(b<=a&&b!=1440)b+=1440;std::time_t n=std::time(nullptr);std::tm u{};gmtime_s(&u,&n);std::time_t z=n-u.tm_hour*3600-u.tm_min*60-u.tm_sec;auto fmt=[z](int m){std::time_t v=z+m*60;std::tm l{};localtime_s(&l,&v);std::ostringstream o;o<<std::put_time(&l,"%H:%M");return o.str();};return fmt(a)+" - "+fmt(b);}
std::string drmScheduleFrequency(double f){std::ostringstream o;o<<std::fixed<<std::setprecision(3)<<f;std::string s=o.str();while(!s.empty()&&s.back()=='0')s.pop_back();if(!s.empty()&&s.back()=='.')s.pop_back();return s+"k";}
