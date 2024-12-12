#include "Formatter.h"
#include <nlohmann/json.hpp>
#include <sstream>
#include <iostream>

using json = nlohmann::json;

void addMetaData(const json &data, std::stringstream &output){
    output << data["rootNode"] << "\n";
};

void addErrors(const json &errors, std::stringstream &output){};

std::string format(std::vector<json> results, int lod) {

    std::stringstream output;

    for (const auto &result : results) {
        if (result.contains("errors") && (result["errors"].is_object())) {
            addMetaData(result, output);
            addErrors(result["errors"], output);
        }
    }
    
    std::string outputStr = output.str();
    std::cout << outputStr << std::endl;

    return outputStr;
};
