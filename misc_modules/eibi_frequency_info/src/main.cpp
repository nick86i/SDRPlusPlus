#include <utils/net.h>
#include <utils/proto/http.h>
#include <core.h>
#include <gui/gui.h>
#include <gui/tuner.h>
#include <imgui.h>
#include <module.h>
#include <signal_path/signal_path.h>
#include <utils/flog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

SDRPP_MOD_INFO{
    "eibi_frequency_info",
    "EiBi schedule information for the currently tuned frequency",
    "Nick / OpenAI",
    0, 1, 0,
    1
};

namespace {
    constexpr const char* EIBI_HOST = "www.eibispace.de";
    constexpr double MATCH_TOLERANCE_HZ = 2500.0;

    struct Entry {
        double frequencyKHz = 0.0;
        std::string time;
        std::string days;
        std::string country;
        std::string station;
        std::string language;
        std::string target;
        std::string remarks;
        std::string period;
        std::string startDate;
        std::string stopDate;
    };

    std::string targetName(const std::string& code) {
        struct TargetName { const char* code; const char* name; };
        static const TargetName names[] = {
            { "Af", "Africa" },
            { "Am", "Americas" },
            { "As", "Asia" },
            { "Car", "Caribbean, Gulf of Mexico and Florida waters" },
            { "Cau", "Caucasus" },
            { "CIS", "Commonwealth of Independent States" },
            { "CNA", "Central North America" },
            { "ENA", "Eastern North America" },
            { "ENE", "East-northeast" },
            { "ESE", "East-southeast" },
            { "Eu", "Europe" },
            { "FE", "Far East" },
            { "Glo", "Global" },
            { "In", "Indian subcontinent" },
            { "LAm", "Latin America" },
            { "ME", "Middle East" },
            { "NAO", "North Atlantic Ocean" },
            { "NE", "Northeast" },
            { "NNE", "North-northeast" },
            { "NNW", "North-northwest" },
            { "NW", "Northwest" },
            { "Oc", "Oceania" },
            { "SAO", "South Atlantic Ocean" },
            { "SE", "Southeast" },
            { "SEA", "Southeast Asia" },
            { "SEE", "Southeastern Europe" },
            { "Sib", "Siberia" },
            { "SSE", "South-southeast" },
            { "SSW", "South-southwest" },
            { "SW", "Southwest" },
            { "Tib", "Tibet" },
            { "WIO", "Western Indian Ocean" },
            { "WNA", "Western North America" },
            { "WNW", "West-northwest" },
            { "WSW", "West-southwest" }
        };
        for (const auto& item : names) {
            if (code == item.code) return item.name;
        }

        // EiBi also constructs regions from a direction plus a continent,
        // for example WEu (Western Europe), NAf and SAs.
        if (code.size() >= 3) {
            const std::string directionCode = code.substr(0, code.size() - 2);
            const std::string areaCode = code.substr(code.size() - 2);
            std::string direction;
            if (directionCode == "C") direction = "Central";
            else if (directionCode == "E") direction = "Eastern";
            else if (directionCode == "N") direction = "Northern";
            else if (directionCode == "S") direction = "Southern";
            else if (directionCode == "W") direction = "Western";

            std::string area;
            if (areaCode == "Af") area = "Africa";
            else if (areaCode == "Am") area = "America";
            else if (areaCode == "As") area = "Asia";
            else if (areaCode == "Eu") area = "Europe";
            else if (areaCode == "In") area = "Indian subcontinent";
            else if (areaCode == "Oc") area = "Oceania";
            if (!direction.empty() && !area.empty()) return direction + " " + area;
        }
        return code;
    }

    std::string transmitterName(const std::string& homeCountry, const std::string& siteCode) {
        // Transmitter-site codes are scoped by country in the EiBi README.
        // A /CCC-code value means that the transmitter is in a host country.
        std::string country = homeCountry;
        std::string code = siteCode;
        if (!code.empty() && code[0] == '/') {
            const size_t dash = code.find('-');
            country = code.substr(1, dash == std::string::npos ? std::string::npos : dash - 1);
            code = dash == std::string::npos ? "" : code.substr(dash + 1);
        }

        struct SiteName { const char* country; const char* code; const char* name; };
        static const SiteName sites[] = {
            { "ALB", "s", "Shijiak, Albania" },
            { "ALG", "s", "In Salah, Algeria" },
            { "AZE", "s", "Stepanakert" },
            { "AUS", "s", "Shepparton, Australia" },
            { "BGD", "s", "Dhaka-Savar, Bangladesh" },
            { "BUL", "s", "Sofia-Kostinbrod, Bulgaria" },
            { "CHL", "s", "Santiago (Calera de Tango), Chile" },
            { "CHN", "s", "Shijiazhuang, China" },
            { "CLM", "pl", "Puerto Lleras, Colombia" },
            { "CZE", "pl", "Praha-Liblice, Czechia" },
            { "D", "s", "Stade, Germany" },
            { "EQA", "s", "Saraguro, Ecuador" },
            { "G", "s", "Skelton, United Kingdom" },
            { "INS", "pl", "Plaju, Palembang, Indonesia" },
            { "IRL", "s", "Shannon, Ireland" },
            { "IRN", "s", "Sirjan, Iran" },
            { "IRQ", "s", "Sulaimaniya, Iraq" },
            { "ISL", "s", "Saudanes, Iceland" },
            { "KOR", "s", "Seoul-Incheon, South Korea" },
            { "KRE", "s", "Sariwon, North Korea" },
            { "LBR", "s", "Star Radio, Monrovia, Liberia" },
            { "MEX", "s", "San Luis Potosi, Mexico" },
            { "MLA", "s", "Sibu, Malaysia" },
            { "MRA", "s", "Saipan/Agingan Point, Northern Mariana Islands" },
            { "MRC", "s", "Safi, Morocco" },
            { "OMA", "s", "Seeb, Oman" },
            { "PTR", "s", "Salinas, Puerto Rico" },
            { "ROU", "s", "Saftica, Romania" },
            { "RUS", "s", "Samara (Zhygulevsk), Russia" },
            { "S", "s", "Sala, Sweden" },
            { "SRB", "s", "Stubline, Serbia" },
            { "TKM", "s", "Seyda, Turkmenistan" },
            { "TUN", "s", "Sfax, Tunisia" },
            { "TWN", "s", "Danshui (Tamsui), Taiwan" },
            { "UZB", "s", "Samarkand, Uzbekistan" },
            { "VTN", "s", "Hanoi-Sontay, Vietnam" },
            { "YEM", "s", "Sanaa, Yemen" }
        };
        for (const auto& site : sites) {
            if (country == site.country && code == site.code) return site.name;
        }
        if (code.empty() && country != homeCountry) return country;
        return "Site code " + siteCode;
    }

    std::string languageName(const std::string& code) {
        struct LanguageName { const char* code; const char* name; };
        static const LanguageName languages[] = {
            { "A", "Arabic" }, { "AFG", "Pashto and Dari" }, { "AH", "Amharic" },
            { "AL", "Albanian" }, { "AM", "Amoy" }, { "AMD", "Tibetan Amdo" },
            { "AR", "Armenian" }, { "AZ", "Azerbaijani" }, { "BE", "Bengali" },
            { "BM", "Bambara" }, { "BR", "Burmese" }, { "BSL", "Bislama" },
            { "BU", "Bulgarian" }, { "CA", "Cantonese" }, { "CR", "Haitian Creole" },
            { "CZ", "Czech" }, { "D", "German" }, { "DR", "Dari" },
            { "E", "English" }, { "EO", "Esperanto" }, { "F", "French" },
            { "FI", "Finnish" }, { "FS", "Farsi" }, { "FU", "Fulani" },
            { "GR", "Greek" }, { "HA", "Hausa" }, { "HI", "Hindi" },
            { "HK", "Hakka" }, { "HR", "Croatian" }, { "HU", "Hungarian" },
            { "I", "Italian" }, { "IN", "Indonesian" }, { "J", "Japanese" },
            { "JV", "Javanese" }, { "K", "Korean" }, { "KA", "Karen" },
            { "KG", "Kyrgyz" }, { "KH", "Khmer" }, { "KHA", "Eastern Kham Tibetan" },
            { "KNK", "Kinyarwanda and Kirundi" }, { "KNU", "Kanuri" },
            { "KU", "Kurdish" }, { "KZ", "Kazakh" }, { "LAO", "Lao" },
            { "M", "Mandarin Chinese" }, { "ML", "Malay" }, { "MO", "Mongolian" },
            { "NE", "Nepali" }, { "NL", "Dutch" }, { "NO", "Norwegian" },
            { "OO", "Oromo" }, { "OR", "Odia" }, { "P", "Portuguese" },
            { "PJ", "Punjabi" }, { "PO", "Polish" }, { "PS", "Pashto" },
            { "Q", "Quechua" }, { "R", "Russian" }, { "RO", "Romanian" },
            { "RU", "Rusyn" }, { "S", "Spanish" }, { "SEF", "Ladino" },
            { "SHA", "Shan" }, { "SLM", "Solomon Islands Pijin" },
            { "SO", "Somali" }, { "SR", "Serbian" }, { "SUD", "Sudanese Arabic" },
            { "SV", "Slovenian" }, { "SWA", "Swahili" }, { "T", "Thai" },
            { "TAG", "Tagalog" }, { "TAM", "Tamil" }, { "TB", "Tibetan" },
            { "TEL", "Telugu" }, { "TIG", "Tigrinya" }, { "TJ", "Tajik" },
            { "TK", "Turkmen" }, { "TO", "Tongan" }, { "TP", "Tok Pisin" },
            { "TU", "Turkish" }, { "UI", "Uighur" }, { "UK", "Ukrainian" },
            { "UR", "Urdu" }, { "VN", "Vernacular/local language" },
            { "-CW", "Morse code" }, { "-HF", "HFDL data" },
            { "-TS", "Time signal" }, { "-TY", "Radioteletype" }
        };

        const size_t comma = code.find(',');
        if (comma != std::string::npos) {
            return languageName(code.substr(0, comma)) + ", " + languageName(code.substr(comma + 1));
        }
        for (const auto& language : languages) {
            if (code == language.code) return language.name;
        }
        return code;
    }

    std::vector<std::string> splitCsv(const std::string& line) {
        std::vector<std::string> fields;
        std::string field;
        bool quoted = false;
        for (size_t i = 0; i < line.size(); ++i) {
            const char c = line[i];
            if (c == '"') {
                if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                    field.push_back('"');
                    ++i;
                }
                else {
                    quoted = !quoted;
                }
            }
            else if (c == ';' && !quoted) {
                fields.push_back(field);
                field.clear();
            }
            else if (c != '\r') {
                field.push_back(c);
            }
        }
        fields.push_back(field);
        return fields;
    }

    std::vector<Entry> parseDatabase(const std::string& data) {
        std::vector<Entry> result;
        std::istringstream stream(data);
        std::string line;
        bool header = true;
        while (std::getline(stream, line)) {
            if (header) { header = false; continue; }
            if (line.empty()) continue;
            const auto f = splitCsv(line);
            if (f.size() < 8) continue;
            try {
                Entry e;
                e.frequencyKHz = std::stod(f[0]);
                e.time = f[1];
                e.days = f[2];
                e.country = f[3];
                e.station = f[4];
                e.language = f[5];
                e.target = f[6];
                e.remarks = f[7];
                if (f.size() > 8) e.period = f[8];
                if (f.size() > 9) e.startDate = f[9];
                if (f.size() > 10) e.stopDate = f[10];
                result.push_back(std::move(e));
            }
            catch (...) {}
        }
        std::sort(result.begin(), result.end(), [](const Entry& a, const Entry& b) {
            return a.frequencyKHz < b.frequencyKHz;
        });
        return result;
    }

    int dayToken(const std::string& token) {
        static const char* names[] = { "Mo", "Tu", "We", "Th", "Fr", "Sa", "Su" };
        for (int i = 0; i < 7; ++i) {
            if (token == names[i]) return i + 1;
        }
        return 0;
    }

    bool dayRangeContains(const std::string& token, int weekday) {
        const size_t dash = token.find('-');
        if (dash != std::string::npos) {
            const int first = dayToken(token.substr(0, dash));
            const int last = dayToken(token.substr(dash + 1));
            if (first && last) {
                return first <= last ? (weekday >= first && weekday <= last)
                                     : (weekday >= first || weekday <= last);
            }
        }
        if (dayToken(token) == weekday) return true;
        for (size_t i = 0; i + 1 < token.size(); i += 2) {
            if (dayToken(token.substr(i, 2)) == weekday) return true;
        }
        return false;
    }

    bool activeOnDay(const std::string& days, int weekday, int monthDay, int month) {
        if (days.empty()) return true;
        if (std::all_of(days.begin(), days.end(), [](unsigned char c) { return std::isdigit(c); })) {
            return days.find(static_cast<char>('0' + weekday)) != std::string::npos;
        }

        static const char* months[] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
        for (int m = 0; m < 12; ++m) {
            const size_t pos = days.find(months[m]);
            if (pos != std::string::npos) {
                try { return std::stoi(days.substr(0, pos)) == monthDay && month == m + 1; }
                catch (...) { return false; }
            }
        }

        if (days.rfind("MF", 0) == 0) return weekday >= 1 && weekday <= 5;
        if (days.find("irr") != std::string::npos || days.find("test") != std::string::npos ||
            days.find("Test") != std::string::npos || days.find("tent") != std::string::npos) return true;

        std::istringstream parts(days);
        std::string part;
        while (std::getline(parts, part, ',')) {
            if (dayRangeContains(part, weekday)) return true;
        }
        return false;
    }

    bool dateInRange(const Entry& entry, int monthDay, int month) {
        if (entry.period != "6" || (entry.startDate.empty() && entry.stopDate.empty())) return true;
        auto parse = [](const std::string& value) {
            if (value.size() < 4 || !std::isdigit(static_cast<unsigned char>(value[0]))) return -1;
            try {
                const int day = std::stoi(value.substr(0, 2));
                const int mon = std::stoi(value.substr(2, 2));
                return mon * 100 + day;
            }
            catch (...) { return -1; }
        };
        const int now = month * 100 + monthDay;
        const int start = parse(entry.startDate);
        const int stop = parse(entry.stopDate);
        if (start < 0 || stop < 0) return true;
        return start <= stop ? (now >= start && now <= stop) : (now >= start || now <= stop);
    }

    bool isActiveNow(const Entry& entry, const std::tm& utc) {
        if (entry.time.size() < 9 || entry.time[4] != '-') return false;
        auto minutes = [](const std::string& value) {
            try { return std::stoi(value.substr(0, 2)) * 60 + std::stoi(value.substr(2, 2)); }
            catch (...) { return -1; }
        };
        const int start = minutes(entry.time.substr(0, 4));
        const int stop = minutes(entry.time.substr(5, 4));
        if (start < 0 || stop < 0) return false;
        const int now = utc.tm_hour * 60 + utc.tm_min;
        const bool wraps = stop <= start && stop != 1440;
        const bool timeActive = wraps ? (now >= start || now < stop) : (now >= start && now < stop);
        if (!timeActive) return false;

        int weekday = utc.tm_wday == 0 ? 7 : utc.tm_wday;
        if (wraps && now < stop) weekday = weekday == 1 ? 7 : weekday - 1;
        return activeOnDay(entry.days, weekday, utc.tm_mday, utc.tm_mon + 1) &&
               dateInRange(entry, utc.tm_mday, utc.tm_mon + 1);
    }

    std::string localTimeRange(const Entry& entry) {
        if (entry.time.size() < 9 || entry.time[4] != '-') return entry.time;
        auto minutes = [](const std::string& value) {
            try { return std::stoi(value.substr(0, 2)) * 60 + std::stoi(value.substr(2, 2)); }
            catch (...) { return -1; }
        };
        int start = minutes(entry.time.substr(0, 4));
        int stop = minutes(entry.time.substr(5, 4));
        if (start < 0 || stop < 0) return entry.time;
        if (stop <= start && stop != 1440) stop += 1440;

        const std::time_t now = std::time(nullptr);
        std::tm utc{};
#ifdef _WIN32
        gmtime_s(&utc, &now);
#else
        gmtime_r(&now, &utc);
#endif
        const std::time_t utcMidnight = now - utc.tm_hour * 3600 - utc.tm_min * 60 - utc.tm_sec;
        auto formatLocal = [utcMidnight](int minute) {
            const std::time_t value = utcMidnight + static_cast<std::time_t>(minute) * 60;
            std::tm local{};
#ifdef _WIN32
            localtime_s(&local, &value);
#else
            localtime_r(&value, &local);
#endif
            std::ostringstream formatted;
            formatted << std::put_time(&local, "%H:%M");
            return formatted.str();
        };
        return formatLocal(start) + " - " + formatLocal(stop);
    }

    std::string frequencyLabel(double frequencyKHz) {
        std::ostringstream formatted;
        formatted << std::fixed << std::setprecision(3) << frequencyKHz;
        std::string value = formatted.str();
        while (!value.empty() && value.back() == '0') value.pop_back();
        if (!value.empty() && value.back() == '.') value.pop_back();
        return value + " kHz";
    }

    std::string compactFrequencyLabel(double frequencyKHz) {
        std::ostringstream formatted;
        formatted << std::fixed << std::setprecision(3) << frequencyKHz;
        std::string value = formatted.str();
        while (!value.empty() && value.back() == '0') value.pop_back();
        if (!value.empty() && value.back() == '.') value.pop_back();
        return value + "k";
    }

    std::string currentSeasonPath() {
        const std::time_t now = std::time(nullptr);
        std::tm utc{};
#ifdef _WIN32
        gmtime_s(&utc, &now);
#else
        gmtime_r(&now, &utc);
#endif
        int year = utc.tm_year + 1900;
        const int month = utc.tm_mon + 1;
        char season = 'a';
        if (month >= 10) season = 'b';
        else if (month <= 2) { season = 'b'; --year; }
        char path[64];
        std::snprintf(path, sizeof(path), "/dx/sked-%c%02d.csv", season, year % 100);
        return path;
    }

    bool downloadHttp(const std::string& path, std::string& body, std::string& error) {
        try {
            auto socket = net::connect(EIBI_HOST, 80);
            if (!socket || !socket->isOpen()) {
                error = "Could not connect to EiBi";
                return false;
            }
            net::http::Client client(socket);
            net::http::RequestHeader request(net::http::METHOD_GET, path, EIBI_HOST);
            request.setField("Connection", "close");
            request.setField("User-Agent", "SDR++ EiBi Frequency Info/0.1");
            if (client.sendRequestHeader(request) < 0) {
                error = "Could not send HTTP request";
                return false;
            }
            net::http::ResponseHeader response;
            if (client.recvResponseHeader(response, 10000) < 0) {
                error = "EiBi response timed out";
                return false;
            }
            if (response.getStatusCode() != net::http::STATUS_CODE_OK) {
                error = "EiBi returned " + std::to_string((int)response.getStatusCode());
                return false;
            }

            body.clear();
            if (response.hasField("Content-Length")) {
                const size_t length = std::stoull(response.getField("Content-Length"));
                body.resize(length);
                const int received = socket->recv(reinterpret_cast<uint8_t*>(body.data()), length, true, 15000);
                if (received != static_cast<int>(length)) {
                    error = "Incomplete EiBi download";
                    return false;
                }
            }
            else {
                uint8_t buffer[16384];
                while (true) {
                    const int received = socket->recv(buffer, sizeof(buffer), false, 15000);
                    if (received <= 0) break;
                    body.append(reinterpret_cast<const char*>(buffer), received);
                }
            }
            return !body.empty();
        }
        catch (const std::exception& ex) {
            error = ex.what();
            return false;
        }
    }

    std::string discoverCurrentDatabasePath() {
        std::string homepage;
        std::string ignoredError;
        if (downloadHttp("/", homepage, ignoredError)) {
            const size_t pos = homepage.find("sked-");
            if (pos != std::string::npos && pos + 12 <= homepage.size()) {
                const std::string fileName = homepage.substr(pos, 12);
                if ((fileName[5] == 'a' || fileName[5] == 'b') &&
                    std::isdigit(static_cast<unsigned char>(fileName[6])) &&
                    std::isdigit(static_cast<unsigned char>(fileName[7])) &&
                    fileName.substr(8) == ".csv") {
                    return "/dx/" + fileName;
                }
            }
        }
        return currentSeasonPath();
    }
}

class EiBiFrequencyInfoModule : public ModuleManager::Instance {
public:
    explicit EiBiFrequencyInfoModule(std::string instanceName) : name(std::move(instanceName)) {
        cachePath = core::args["root"].s() + "/eibi_schedule.csv";
        loadCache();
        gui::menu.registerEntry(name, menuHandler, this, this);
        requestRefresh();
    }

    ~EiBiFrequencyInfoModule() override {
        gui::menu.removeEntry(name);
        if (downloadThread.joinable()) downloadThread.join();
    }

    void postInit() override {}
    void enable() override { enabled = true; }
    void disable() override { enabled = false; }
    bool isEnabled() override { return enabled; }

private:
    void loadCache() {
        std::ifstream file(cachePath, std::ios::binary);
        if (!file) return;
        std::ostringstream data;
        data << file.rdbuf();
        auto parsed = parseDatabase(data.str());
        if (!parsed.empty()) {
            std::lock_guard<std::mutex> lock(entriesMutex);
            entries = std::move(parsed);
            status = "Loaded cached database";
        }
    }

    void requestRefresh() {
        if (downloading.exchange(true)) return;
        if (downloadThread.joinable()) downloadThread.join();
        {
            std::lock_guard<std::mutex> lock(entriesMutex);
            status = "Downloading current EiBi database...";
            showUpdateAge = false;
        }
        downloadThread = std::thread([this]() {
            std::string data;
            std::string error;
            const std::string path = discoverCurrentDatabasePath();
            if (downloadHttp(path, data, error)) {
                auto parsed = parseDatabase(data);
                if (!parsed.empty()) {
                    size_t loadedCount = 0;
                    {
                        std::lock_guard<std::mutex> lock(entriesMutex);
                        entries = std::move(parsed);
                        loadedCount = entries.size();
                        databasePath = path;
                        status = "Updated";
                        lastUpdated = std::time(nullptr);
                        showUpdateAge = true;
                    }
                    std::ofstream cache(cachePath, std::ios::binary | std::ios::trunc);
                    if (cache) cache.write(data.data(), static_cast<std::streamsize>(data.size()));
                    flog::info("EiBi Frequency Info: loaded {} schedule entries from {}", loadedCount, path);
                }
                else {
                    std::lock_guard<std::mutex> lock(entriesMutex);
                    status = "Downloaded database could not be parsed";
                    showUpdateAge = false;
                }
            }
            else {
                std::lock_guard<std::mutex> lock(entriesMutex);
                status = "Update failed: " + error;
                showUpdateAge = false;
                flog::error("EiBi Frequency Info: {}", status);
            }
            downloading = false;
        });
    }

    double tunedFrequency() const {
        double frequency = gui::waterfall.getCenterFrequency();
        if (!gui::waterfall.selectedVFO.empty() && sigpath::vfoManager.vfoExists(gui::waterfall.selectedVFO)) {
            frequency += sigpath::vfoManager.getOffset(gui::waterfall.selectedVFO);
        }
        return frequency;
    }

    static void menuHandler(void* ctx) {
        auto* self = static_cast<EiBiFrequencyInfoModule*>(ctx);
        const double frequencyHz = self->tunedFrequency();
        const double frequencyKHz = frequencyHz / 1000.0;

        std::vector<Entry> matches;
        std::vector<Entry> drmEntries;
        std::string status;
        std::time_t lastUpdated = 0;
        bool showUpdateAge = false;
        size_t totalEntries = 0;
        {
            std::lock_guard<std::mutex> lock(self->entriesMutex);
            status = self->status;
            lastUpdated = self->lastUpdated;
            showUpdateAge = self->showUpdateAge;
            totalEntries = self->entries.size();
            const double toleranceKHz = MATCH_TOLERANCE_HZ / 1000.0;
            const auto first = std::lower_bound(self->entries.begin(), self->entries.end(), frequencyKHz - toleranceKHz,
                [](const Entry& e, double value) { return e.frequencyKHz < value; });
            for (auto it = first; it != self->entries.end() && it->frequencyKHz <= frequencyKHz + toleranceKHz; ++it) {
                matches.push_back(*it);
            }
            for (const auto& entry : self->entries) {
                if (entry.frequencyKHz >= 2300.0 && entry.frequencyKHz <= 30000.0 &&
                    entry.station.find("DIGITAL") != std::string::npos) drmEntries.push_back(entry);
            }
        }

        if (self->downloading) {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "%s", status.c_str());
        }
        else if (showUpdateAge && lastUpdated > 0) {
            const auto elapsedSeconds = (std::max)(0LL,
                static_cast<long long>(std::difftime(std::time(nullptr), lastUpdated)));
            if (elapsedSeconds < 3600) {
                ImGui::Text("Updated %lld min ago (%zu entries)", elapsedSeconds / 60, totalEntries);
            }
            else {
                ImGui::Text("Updated %lld hrs ago (%zu entries)", elapsedSeconds / 3600, totalEntries);
            }
        }
        else {
            ImGui::TextWrapped("%s (%zu entries)", status.c_str(), totalEntries);
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh##eibi")) self->requestRefresh();
        ImGui::Separator();

        if (matches.empty()) {
            ImGui::TextWrapped("No EiBi entries within +/-2.5 kHz.");
        }

        const std::time_t now = std::time(nullptr);
        std::tm utc{};
#ifdef _WIN32
        gmtime_s(&utc, &now);
#else
        gmtime_r(&now, &utc);
#endif
        std::vector<Entry> active;
        std::vector<Entry> other;
        for (const auto& entry : matches) {
            (isActiveNow(entry, utc) ? active : other).push_back(entry);
        }

        if (active.empty()) {
            ImGui::TextUnformatted("No current broadcast");
        }
        bool firstActiveEntry = true;
        for (const auto& entry : active) {
            if (!firstActiveEntry) ImGui::Separator();
            firstActiveEntry = false;
            const std::string compactFrequency = compactFrequencyLabel(entry.frequencyKHz);
            const std::string scheduleTime = localTimeRange(entry);
            ImGui::PushID(&entry);
            if (ImGui::BeginTable("CurrentBroadcastHeader", 3,
                                  ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX)) {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed,
                                        ImGui::CalcTextSize(compactFrequency.c_str()).x);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed,
                                        ImGui::CalcTextSize("00:00 - 00:00").x);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextWrapped("%s", entry.station.empty() ? "(unnamed station)" : entry.station.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(compactFrequency.c_str());
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(scheduleTime.c_str());
                ImGui::EndTable();
            }
            ImGui::PopID();
            if (!entry.country.empty() || !entry.language.empty()) {
                const std::string decodedLanguage = languageName(entry.language);
                ImGui::TextWrapped("Country: %s   Language: %s", entry.country.c_str(), decodedLanguage.c_str());
            }
            if (!entry.days.empty()) ImGui::TextWrapped("Days: %s", entry.days.c_str());
            if (!entry.remarks.empty() && !entry.target.empty()) {
                const std::string transmitter = transmitterName(entry.country, entry.remarks);
                const std::string decodedTarget = targetName(entry.target);
                ImGui::TextWrapped("%s -> %s", transmitter.c_str(), decodedTarget.c_str());
            }
            else if (!entry.remarks.empty()) {
                const std::string transmitter = transmitterName(entry.country, entry.remarks);
                ImGui::TextWrapped("%s", transmitter.c_str());
            }
            else if (!entry.target.empty()) {
                const std::string decodedTarget = targetName(entry.target);
                ImGui::TextWrapped("%s", decodedTarget.c_str());
            }
            if (!entry.startDate.empty() || !entry.stopDate.empty()) {
                ImGui::TextWrapped("Dates: %s - %s", entry.startDate.c_str(), entry.stopDate.c_str());
            }
        }

        ImGui::Separator();
        ImGui::Checkbox("Show Other Broadcasts##eibi", &self->showOtherBroadcasts);
        if (self->showOtherBroadcasts && !other.empty()) {
                ImGui::BeginTable("EiBiOtherBroadcasts", 2,
                                  ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed,
                                        ImGui::CalcTextSize("00:00 - 00:00").x);
                for (const auto& entry : other) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextWrapped("%s", entry.station.empty() ? "(unnamed station)" : entry.station.c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::TextUnformatted(localTimeRange(entry).c_str());
                }
                ImGui::EndTable();
        }
        ImGui::Checkbox("Show DRM Broadcasts##eibi", &self->showDrmBroadcasts);
        if (self->showDrmBroadcasts) {
            std::vector<double> activeDrmFrequencies;
            for (const auto& entry : drmEntries) {
                if (isActiveNow(entry, utc)) activeDrmFrequencies.push_back(entry.frequencyKHz);
            }
            std::sort(activeDrmFrequencies.begin(), activeDrmFrequencies.end());
            activeDrmFrequencies.erase(std::unique(activeDrmFrequencies.begin(), activeDrmFrequencies.end()),
                                       activeDrmFrequencies.end());
            const auto nextIt = std::upper_bound(activeDrmFrequencies.begin(), activeDrmFrequencies.end(), frequencyKHz);
            const auto previousIt = std::lower_bound(activeDrmFrequencies.begin(), activeDrmFrequencies.end(), frequencyKHz);
            const bool hasPrevious = previousIt != activeDrmFrequencies.begin();
            const bool hasNext = nextIt != activeDrmFrequencies.end();
            auto tuneDrmFrequency = [](double targetKHz) {
                const std::string activeVfo = gui::waterfall.selectedVFO;
                if (!activeVfo.empty() && sigpath::vfoManager.vfoExists(activeVfo))
                    tuner::normalTuning(activeVfo, targetKHz * 1000.0);
                else tuner::iqTuning(targetKHz * 1000.0);
            };
            ImGui::SameLine();
            if (!hasPrevious) ImGui::BeginDisabled();
            if (ImGui::Button("<##previous_drm")) tuneDrmFrequency(*std::prev(previousIt));
            if (!hasPrevious) ImGui::EndDisabled();
            ImGui::SameLine();
            if (!hasNext) ImGui::BeginDisabled();
            if (ImGui::Button(">##next_drm")) tuneDrmFrequency(*nextIt);
            if (!hasNext) ImGui::EndDisabled();

            bool anyDrm = false;
            if (ImGui::BeginTable("EiBiCurrentDrm", 3,
                    ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoPadOuterX)) {
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("17870k").x);
                ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, ImGui::CalcTextSize("00:00 - 00:00").x);
                int rowId = 0;
                for (auto entry : drmEntries) {
                    if (!isActiveNow(entry, utc)) continue;
                    anyDrm = true;
                    const auto suffix = entry.station.rfind(" DIGITAL");
                    if (suffix != std::string::npos) entry.station.erase(suffix);
                    const std::string frequency = compactFrequencyLabel(entry.frequencyKHz);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(entry.station.c_str());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::PushID(rowId++);
                    ImGui::TextUnformatted(frequency.c_str());
                    const bool clicked = ImGui::IsItemClicked();
                    const ImVec2 itemMin = ImGui::GetItemRectMin(), itemMax = ImGui::GetItemRectMax();
                    ImGui::GetWindowDrawList()->AddLine(ImVec2(itemMin.x, itemMax.y), ImVec2(itemMax.x, itemMax.y),
                        ImGui::GetColorU32(ImGui::IsItemHovered() ? ImGuiCol_TextSelectedBg : ImGuiCol_Text));
                    if (clicked) {
                        const std::string activeVfo = gui::waterfall.selectedVFO;
                        if (!activeVfo.empty() && sigpath::vfoManager.vfoExists(activeVfo))
                            tuner::normalTuning(activeVfo, entry.frequencyKHz * 1000.0);
                        else tuner::iqTuning(entry.frequencyKHz * 1000.0);
                    }
                    ImGui::PopID();
                    ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(localTimeRange(entry).c_str());
                }
                ImGui::EndTable();
            }
            if (!anyDrm) ImGui::TextDisabled("No scheduled DRM broadcasts");
        }
    }

    std::string name;
    std::string cachePath;
    std::string databasePath;
    std::string status = "No database loaded";
    std::time_t lastUpdated = 0;
    bool showUpdateAge = false;
    bool enabled = true;
    bool showOtherBroadcasts = false;
    bool showDrmBroadcasts = false;
    std::atomic<bool> downloading{false};
    std::thread downloadThread;
    std::mutex entriesMutex;
    std::vector<Entry> entries;
};

MOD_EXPORT void _INIT_() {}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new EiBiFrequencyInfoModule(std::move(name));
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete static_cast<EiBiFrequencyInfoModule*>(instance);
}

MOD_EXPORT void _END_() {}
