#include <iostream>
#include <curl/curl.h>
#include <string>
#include <map>
#include <json/json.h> 
#include <sstream>

// Helper function to convert std::map to JSON format string
std::string mapToJson(const std::map<std::string, std::string>& params) {
    std::string json = "{";
    for (const auto& pair : params) {
        json += "\"" + pair.first + "\": ";
        if (pair.second.find_first_not_of("0123456789") == std::string::npos) {
            json += pair.second; // Treat as number if it contains only digits
        } else {
            json += "\"" + pair.second + "\""; // Treat as string otherwise
        }
        json += ", ";
    }
    if (json.size() > 1) json.pop_back(), json.pop_back(); // Remove last comma and space
    json += "}";
    return json;
}

size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

int main() {
    // Configuration
    std::string openai_api_key = "sk-wjI23SNEp3giHqfy948979B05eA0499bBa68604959888403";
    std::string openai_base_url = "https://api.vveai.com/v1/chat/completions";
    std::map<std::string, std::string> openai_default_headers = {{"x-foo", "true"}};

    std::string user_input;
    std::cout << "input your command: ";
    std::getline(std::cin, user_input);  // 从标准输入读取一行用户输入

    std::string prompt;
    std::string user_query;
    prompt = R"(prompt: You should determine whether hot or cold water should be connected according to the user's request, and output \"hot\" if you think hot water should be connected, otherwise output \"cool\".For example: # I just finished exercising, I'm thirsty. Output: cool#I have an upset stomach. Output: hotThis is my request: #)";
    user_query = prompt + user_input;
    std::cout << user_query;
    // Request parameters
    std::string json_data = R"({
        "model": "gpt-4o",
        "messages": [
            {"role": "user", "content": ")" + user_query + R"("}
        ],
        "max_tokens": 200,
        "temperature": 0
    })";

    std::cout << "发送的 JSON 数据: " << json_data << std::endl;

    CURL* curl = curl_easy_init();
    if (curl) {
        struct curl_slist* headers = nullptr;

        // Add headers
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, ("Authorization: Bearer " + openai_api_key).c_str());
        for (const auto& header : openai_default_headers) {
            headers = curl_slist_append(headers, (header.first + ": " + header.second).c_str());
        }

        // Set CURL options
        curl_easy_setopt(curl, CURLOPT_URL, openai_base_url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        //std::string json_data = mapToJson(params);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        // Perform the request
        // CURLcode res = curl_easy_perform(curl);
        // if (res == CURLE_OK) {
        //     long status_code = 0;
        //     curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
        //     if (status_code == 200) {
        //         std::cout << "Request succeeded!" << std::endl;
        //     } else {
        //         std::cerr << "HTTP Error: " << status_code << std::endl;
        //     }
        // } else {
        //     std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        // }
        std::string response_data;
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);

        CURLcode res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            long status_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
            if (status_code == 200) {
                // Parse JSON response
                Json::CharReaderBuilder reader;
                Json::Value root;
                std::istringstream s(response_data);
                std::string errs;
                if (Json::parseFromStream(reader, s, &root, &errs)) {
                    // Extract the content from the first choice
                    if (root.isMember("choices") && root["choices"].size() > 0) {
                        std::cout << "Model response: " << root["choices"][0]["message"]["content"].asString() << std::endl;
                    } else {
                        std::cerr << "Error: No choices in the response" << std::endl;
                    }
                } else {
                    std::cerr << "Error parsing JSON: " << errs << std::endl;
                }
            } else {
                std::cerr << "HTTP Error: " << status_code << std::endl;
            }
        } else {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }

        // Cleanup
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }
    return 0;
}