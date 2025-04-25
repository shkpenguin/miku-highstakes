#include "utils.h"
#include "cmath"
#include <vector>
#include <initializer_list>
#include "utils/pose.h"
#include "utils/math.h"

std::string _2f(float num) {
    std::string str = std::to_string(num);
    size_t pos = str.find(".");
    if (pos != std::string::npos) {
        str = str.substr(0, pos + 3);
    }
    return str;
}

std::string _1f(float num) {
    std::string str = std::to_string(num);
    size_t pos = str.find(".");
    if (pos != std::string::npos) {
        str = str.substr(0, pos + 2);
    }
    return str;
}