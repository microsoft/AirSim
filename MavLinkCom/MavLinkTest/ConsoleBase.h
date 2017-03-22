#pragma once

#include <string>
#include "Commands.h"


class ConsoleBase {
public:
    virtual Command* getCommand(const std::string& line) = 0;
    virtual int run() = 0;
};