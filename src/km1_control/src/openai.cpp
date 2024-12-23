#include <iostream>
#include <curl/curl.h>

int main() {
    CURL* curl = curl_easy_init();
    if (curl) {
        const char* url = "https://api.openai.com/v1/engines/davinci-codex/completions";

        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, "Authorization: Bearer YOUR_API_SECRET_KEY");

        const char* json_data = "{\"prompt\": \"Hello, how are you?\", \"temperature\": 0.7, \"max_tokens\": 100}";

        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_data);

        CURLcode res = curl_easy_perform(curl);

        if (res == CURLE_OK) {
            long status_code = 0;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
            if (status_code == 200) {
                std::string response;
                curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response);
                std::cout << response << std::endl;
            } else {
                std::cerr << "HTTP Error: " << status_code << std::endl;
            }
        } else {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
        }

        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    }
    return 0;
}