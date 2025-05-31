#ifndef ELM327_H
#define ELM327_H

#include <string>

struct ELM327
{
    std::string reset = "ATZ";
    std::string no_echo = "ATE0";
    std::string no_line_feed = "ATL0";
    std::string no_spaces = "ATS0";
};

#endif