#pragma once

#include <string>

class Simulator {
public:
    explicit Simulator(const std::string& inputOutputPath);

    int run();

private:
    std::string inputOutputPath;

    std::string makePath(const std::string& fileName) const;
};