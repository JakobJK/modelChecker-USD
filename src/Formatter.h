#ifndef FORMATTER_H
#define FORMATTER_H
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

std::string format(std::vector<json> results, int lod = 0);
void addMetaData(const json &data, std::stringstream &output);
void addErrors(const json &errors, std::stringstream &output);


#endif
