#include <curl/curl.h>

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

size_t write_callback(char* ptr, size_t size, size_t nmemb, void* userdata) {
    auto* out = static_cast<std::string*>(userdata);
    out->append(ptr, size * nmemb);
    return size * nmemb;
}

std::string url_encode(const std::string& value) {
    CURL* curl = curl_easy_init();
    if (!curl) {
        throw std::runtime_error("curl_easy_init failed for url_encode");
    }
    char* escaped = curl_easy_escape(curl, value.c_str(), static_cast<int>(value.size()));
    if (!escaped) {
        curl_easy_cleanup(curl);
        throw std::runtime_error("curl_easy_escape failed");
    }
    std::string out(escaped);
    curl_free(escaped);
    curl_easy_cleanup(curl);
    return out;
}

std::string http_request(const std::string& url, const char* method, const std::string& body = "") {
    CURL* curl = curl_easy_init();
    if (!curl) {
        throw std::runtime_error("curl_easy_init failed");
    }

    struct curl_slist* headers = nullptr;
    std::string response;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

    if (std::string(method) == "POST") {
        headers = curl_slist_append(headers, "Content-Type: application/json");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    }

    CURLcode rc = curl_easy_perform(curl);
    long status = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status);

    if (headers) {
        curl_slist_free_all(headers);
    }
    curl_easy_cleanup(curl);

    if (rc != CURLE_OK) {
        throw std::runtime_error(std::string("curl error: ") + curl_easy_strerror(rc));
    }
    if (status >= 400) {
        throw std::runtime_error("http status " + std::to_string(status) + ": " + response);
    }
    return response;
}

std::string http_get(const std::string& url) {
    return http_request(url, "GET");
}

std::string http_post_json(const std::string& url, const std::string& json_body) {
    return http_request(url, "POST", json_body);
}

std::string last_event_ts(const std::string& events_json) {
    const std::string key = "\"ts\":\"";
    size_t pos = events_json.rfind(key);
    if (pos == std::string::npos) {
        return "";
    }
    pos += key.size();
    size_t end = events_json.find('"', pos);
    if (end == std::string::npos) {
        return "";
    }
    return events_json.substr(pos, end - pos);
}

bool has_event_name(const std::string& events_json, const std::string& name) {
    return events_json.find("\"name\":\"" + name + "\"") != std::string::npos ||
           events_json.find("\"type\":\"" + name + "\"") != std::string::npos;
}

void print_usage(const char* bin) {
    std::cerr << "Usage:\n"
              << "  " << bin << " state [base_url]\n"
              << "  " << bin << " arrived [base_url]\n"
              << "  " << bin << " dock [base_url]\n"
              << "  " << bin << " poll [base_url]\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    const std::string cmd = argv[1];
    const std::string base = (argc >= 3) ? argv[2] : "http://127.0.0.1:5050";

    curl_global_init(CURL_GLOBAL_DEFAULT);
    try {
        if (cmd == "state") {
            std::cout << http_get(base + "/api/state") << "\n";
        } else if (cmd == "arrived") {
            std::cout << http_post_json(base + "/api/mode", R"({"mode":"ARRIVED"})") << "\n";
        } else if (cmd == "dock") {
            std::cout << http_post_json(base + "/api/mode", R"({"mode":"DOCK_IDLE"})") << "\n";
        } else if (cmd == "poll") {
            std::string since_ts;
            while (true) {
                std::string url = base + "/api/events";
                if (!since_ts.empty()) {
                    url += "?since_ts=" + url_encode(since_ts);
                }

                const std::string events = http_get(url);
                if (events != "[]") {
                    std::cout << events << "\n";
                }

                if (has_event_name(events, "start_room1")) {
                    std::cerr << "[external] start ROOM1\n";
                }
                if (has_event_name(events, "start_room2")) {
                    std::cerr << "[external] start ROOM2\n";
                }
                if (has_event_name(events, "power_edge")) {
                    std::cerr << "[external] power edge\n";
                }

                const std::string ts = last_event_ts(events);
                if (!ts.empty()) {
                    since_ts = ts;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(800));
            }
        } else {
            print_usage(argv[0]);
            curl_global_cleanup();
            return 1;
        }
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << "\n";
        curl_global_cleanup();
        return 1;
    }

    curl_global_cleanup();
    return 0;
}
