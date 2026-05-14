#include <string>

#include "simulator.h"

int main(int argc, char* argv[]) {
    std::string inputOutputPath = ".";

    if (argc > 1) {
        inputOutputPath = argv[1];
    }

    Simulator simulator(inputOutputPath);
    return simulator.run();
}