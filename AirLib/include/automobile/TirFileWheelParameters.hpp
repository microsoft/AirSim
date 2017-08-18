#ifndef air_tirfilewheelparameters_hpp
#define air_tirfilewheelparameters_hpp

#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "automobile/WheelParameters.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

    class TirFileWheelParameters : public WheelParameters
    {
        public:
            TirFileWheelParameters() {};

            TirFileWheelParameters(std::string inputFile)
            {
                std::string line;
                std::ifstream inFile(inputFile);

                while (std::getline(inFile, line))
                {
                    //Empty lines and lines beginning with a ! are comments
                    if (line.length() > 0 && line[0] != '!')
                    {
                        std::string identifier_str;
                        std::string value_str;
                        bool foundEqual = false;
                        for (size_t i = 0; i < line.length(); i++)
                        {
                            //There can be arbitrary spacing for human readability
                            if (!isspace(line[i]))
                            {
                                //Characters after a $ are considered part of a comment
                                if (line[i] == '$')
                                {
                                    i = line.length() + 1;
                                }
                                //Separates name from value
                                else if (line[i] == '=')
                                {
                                    foundEqual = true;
                                }
                                //If we haven't seen an equal sign, we are parsing the propery name
                                else if (!foundEqual)
                                {
                                    identifier_str += static_cast<char>(toupper(line[i]));
                                }
                                //After the equal sign, we are parsing the property value
                                else
                                {
                                    value_str += static_cast<char>(toupper(line[i]));
                                }
                            }
                        }

                        //Verify that we have both a property name and a property value
                        if (identifier_str.length() > 0 && value_str.length() > 0)
                        {
                            //Verify that the value can be parsed as a real_T.
                            //It is expected that some properties do not have numerical values.
                            //But, the models do not currently use those properties.
                            real_T value_number;
                            std::istringstream iss(value_str);
                            iss >> value_number;
                            if (iss.eof() && !iss.fail())
                            {
                                /*Although this is ugly, it will give us the best runtime performance.*/
                                /*std::map is O(log_n), and std::unordered_set() is linear in the length of the string.*/
                                if (identifier_str == "UNLOADED_RADIUS")
                                {
                                    UNLOADED_RADIUS = value_number;
                                }
                                if (identifier_str == "WIDTH")
                                {
                                    WIDTH = value_number;
                                }
                                if (identifier_str == "ASPECT_RATIO")
                                {
                                    ASPECT_RATIO = value_number;
                                }
                                if (identifier_str == "RIM_RADIUS")
                                {
                                    RIM_RADIUS = value_number;
                                }
                                if (identifier_str == "RIM_WIDTH")
                                {
                                    RIM_WIDTH = value_number;
                                }
                                if (identifier_str == "VERTICAL_STIFFNESS")
                                {
                                    VERTICAL_STIFFNESS = value_number;
                                }
                                if (identifier_str == "VERTICAL_DAMPING")
                                {
                                    VERTICAL_DAMPING = value_number;
                                }
                                if (identifier_str == "BREFF")
                                {
                                    BREFF = value_number;
                                }
                                if (identifier_str == "DREFF")
                                {
                                    DREFF = value_number;
                                }
                                if (identifier_str == "FREFF")
                                {
                                    FREFF = value_number;
                                }
                                if (identifier_str == "FNOMIN")
                                {
                                    FNOMIN = value_number;
                                }
                                if (identifier_str == "KPUMIN")
                                {
                                    KPUMIN = value_number;
                                }
                                if (identifier_str == "KPUMAX")
                                {
                                    KPUMAX = value_number;
                                }
                                if (identifier_str == "ALPMIN")
                                {
                                    ALPMIN = value_number;
                                }
                                if (identifier_str == "ALPMAX")
                                {
                                    ALPMAX = value_number;
                                }
                                if (identifier_str == "CAMMIN")
                                {
                                    CAMMIN = value_number;
                                }
                                if (identifier_str == "CAMMAX")
                                {
                                    CAMMAX = value_number;
                                }
                                if (identifier_str == "FZMIN")
                                {
                                    FZMIN = value_number;
                                }
                                if (identifier_str == "FZMAX")
                                {
                                    FZMAX = value_number;
                                }
                                if (identifier_str == "LFZO")
                                {
                                    LFZO = value_number;
                                }
                                if (identifier_str == "LCX")
                                {
                                    LCX = value_number;
                                }
                                if (identifier_str == "LMUX")
                                {
                                    LMUX = value_number;
                                }
                                if (identifier_str == "LEX")
                                {
                                    LEX = value_number;
                                }
                                if (identifier_str == "LKX")
                                {
                                    LKX = value_number;
                                }
                                if (identifier_str == "LHX")
                                {
                                    LHX = value_number;
                                }
                                if (identifier_str == "LVX")
                                {
                                    LVX = value_number;
                                }
                                if (identifier_str == "LGAX")
                                {
                                    LGAX = value_number;
                                }
                                if (identifier_str == "LCY")
                                {
                                    LCY = value_number;
                                }
                                if (identifier_str == "LMUY")
                                {
                                    LMUY = value_number;
                                }
                                if (identifier_str == "LEY")
                                {
                                    LEY = value_number;
                                }
                                if (identifier_str == "LKY")
                                {
                                    LKY = value_number;
                                }
                                if (identifier_str == "LHY")
                                {
                                    LHY = value_number;
                                }
                                if (identifier_str == "LVY")
                                {
                                    LVY = value_number;
                                }
                                if (identifier_str == "LGAY")
                                {
                                    LGAY = value_number;
                                }
                                if (identifier_str == "LTR")
                                {
                                    LTR = value_number;
                                }
                                if (identifier_str == "LRES")
                                {
                                    LRES = value_number;
                                }
                                if (identifier_str == "LGAZ")
                                {
                                    LGAZ = value_number;
                                }
                                if (identifier_str == "LXAL")
                                {
                                    LXAL = value_number;
                                }
                                if (identifier_str == "LYKA")
                                {
                                    LYKA = value_number;
                                }
                                if (identifier_str == "LVYKA")
                                {
                                    LVYKA = value_number;
                                }
                                if (identifier_str == "LS")
                                {
                                    LS = value_number;
                                }
                                if (identifier_str == "LSGKP")
                                {
                                    LSGKP = value_number;
                                }
                                if (identifier_str == "LSGAL")
                                {
                                    LSGAL = value_number;
                                }
                                if (identifier_str == "LGYR")
                                {
                                    LGYR = value_number;
                                }
                                if (identifier_str == "LMX")
                                {
                                    LMX = value_number;
                                }
                                if (identifier_str == "LVMX")
                                {
                                    LVMX = value_number;
                                }
                                if (identifier_str == "LMY")
                                {
                                    LMY = value_number;
                                }
                                if (identifier_str == "PCX1")
                                {
                                    PCX1 = value_number;
                                }
                                if (identifier_str == "PDX1")
                                {
                                    PDX1 = value_number;
                                }
                                if (identifier_str == "PDX2")
                                {
                                    PDX2 = value_number;
                                }
                                if (identifier_str == "PDX3")
                                {
                                    PDX3 = value_number;
                                }
                                if (identifier_str == "PEX1")
                                {
                                    PEX1 = value_number;
                                }
                                if (identifier_str == "PEX2")
                                {
                                    PEX2 = value_number;
                                }
                                if (identifier_str == "PEX3")
                                {
                                    PEX3 = value_number;
                                }
                                if (identifier_str == "PEX4")
                                {
                                    PEX4 = value_number;
                                }
                                if (identifier_str == "PKX1")
                                {
                                    PKX1 = value_number;
                                }
                                if (identifier_str == "PKX2")
                                {
                                    PKX2 = value_number;
                                }
                                if (identifier_str == "PKX3")
                                {
                                    PKX3 = value_number;
                                }
                                if (identifier_str == "PHX1")
                                {
                                    PHX1 = value_number;
                                }
                                if (identifier_str == "PHX2")
                                {
                                    PHX2 = value_number;
                                }
                                if (identifier_str == "PVX1")
                                {
                                    PVX1 = value_number;
                                }
                                if (identifier_str == "PVX2")
                                {
                                    PVX2 = value_number;
                                }
                                if (identifier_str == "RBX1")
                                {
                                    RBX1 = value_number;
                                }
                                if (identifier_str == "RBX2")
                                {
                                    RBX2 = value_number;
                                }
                                if (identifier_str == "RCX1")
                                {
                                    RCX1 = value_number;
                                }
                                if (identifier_str == "REX1")
                                {
                                    REX1 = value_number;
                                }
                                if (identifier_str == "REX2")
                                {
                                    REX2 = value_number;
                                }
                                if (identifier_str == "RHX1")
                                {
                                    RHX1 = value_number;
                                }
                                if (identifier_str == "PTX1")
                                {
                                    PTX1 = value_number;
                                }
                                if (identifier_str == "PTX2")
                                {
                                    PTX2 = value_number;
                                }
                                if (identifier_str == "PTX3")
                                {
                                    PTX3 = value_number;
                                }
                                if (identifier_str == "QSX1")
                                {
                                    QSX1 = value_number;
                                }
                                if (identifier_str == "QSX2")
                                {
                                    QSX2 = value_number;
                                }
                                if (identifier_str == "QSX3")
                                {
                                    QSX3 = value_number;
                                }
                                if (identifier_str == "PCY1")
                                {
                                    PCY1 = value_number;
                                }
                                if (identifier_str == "PDY1")
                                {
                                    PDY1 = value_number;
                                }
                                if (identifier_str == "PDY2")
                                {
                                    PDY2 = value_number;
                                }
                                if (identifier_str == "PDY3")
                                {
                                    PDY3 = value_number;
                                }
                                if (identifier_str == "PEY1")
                                {
                                    PEY1 = value_number;
                                }
                                if (identifier_str == "PEY2")
                                {
                                    PEY2 = value_number;
                                }
                                if (identifier_str == "PEY3")
                                {
                                    PEY3 = value_number;
                                }
                                if (identifier_str == "PEY4")
                                {
                                    PEY4 = value_number;
                                }
                                if (identifier_str == "PKY1")
                                {
                                    PKY1 = value_number;
                                }
                                if (identifier_str == "PKY2")
                                {
                                    PKY2 = value_number;
                                }
                                if (identifier_str == "PKY3")
                                {
                                    PKY3 = value_number;
                                }
                                if (identifier_str == "PHY1")
                                {
                                    PHY1 = value_number;
                                }
                                if (identifier_str == "PHY2")
                                {
                                    PHY2 = value_number;
                                }
                                if (identifier_str == "PHY3")
                                {
                                    PHY3 = value_number;
                                }
                                if (identifier_str == "PVY1")
                                {
                                    PVY1 = value_number;
                                }
                                if (identifier_str == "PVY2")
                                {
                                    PVY2 = value_number;
                                }
                                if (identifier_str == "PVY3")
                                {
                                    PVY3 = value_number;
                                }
                                if (identifier_str == "PVY4")
                                {
                                    PVY4 = value_number;
                                }
                                if (identifier_str == "RBY1")
                                {
                                    RBY1 = value_number;
                                }
                                if (identifier_str == "RBY2")
                                {
                                    RBY2 = value_number;
                                }
                                if (identifier_str == "RBY3")
                                {
                                    RBY3 = value_number;
                                }
                                if (identifier_str == "RCY1")
                                {
                                    RCY1 = value_number;
                                }
                                if (identifier_str == "REY1")
                                {
                                    REY1 = value_number;
                                }
                                if (identifier_str == "REY2")
                                {
                                    REY2 = value_number;
                                }
                                if (identifier_str == "RHY1")
                                {
                                    RHY1 = value_number;
                                }
                                if (identifier_str == "RHY2")
                                {
                                    RHY2 = value_number;
                                }
                                if (identifier_str == "RVY1")
                                {
                                    RVY1 = value_number;
                                }
                                if (identifier_str == "RVY2")
                                {
                                    RVY2 = value_number;
                                }
                                if (identifier_str == "RVY3")
                                {
                                    RVY3 = value_number;
                                }
                                if (identifier_str == "RVY4")
                                {
                                    RVY4 = value_number;
                                }
                                if (identifier_str == "RVY5")
                                {
                                    RVY5 = value_number;
                                }
                                if (identifier_str == "RVY6")
                                {
                                    RVY6 = value_number;
                                }
                                if (identifier_str == "PTY1")
                                {
                                    PTY1 = value_number;
                                }
                                if (identifier_str == "PTY2")
                                {
                                    PTY2 = value_number;
                                }
                                if (identifier_str == "QSY1")
                                {
                                    QSY1 = value_number;
                                }
                                if (identifier_str == "QSY2")
                                {
                                    QSY2 = value_number;
                                }
                                if (identifier_str == "QSY3")
                                {
                                    QSY3 = value_number;
                                }
                                if (identifier_str == "QSY4")
                                {
                                    QSY4 = value_number;
                                }
                                if (identifier_str == "QBZ1")
                                {
                                    QBZ1 = value_number;
                                }
                                if (identifier_str == "QBZ2")
                                {
                                    QBZ2 = value_number;
                                }
                                if (identifier_str == "QBZ3")
                                {
                                    QBZ3 = value_number;
                                }
                                if (identifier_str == "QBZ4")
                                {
                                    QBZ4 = value_number;
                                }
                                if (identifier_str == "QBZ5")
                                {
                                    QBZ5 = value_number;
                                }
                                if (identifier_str == "QBZ9")
                                {
                                    QBZ9 = value_number;
                                }
                                if (identifier_str == "QBZ10")
                                {
                                    QBZ10 = value_number;
                                }
                                if (identifier_str == "QCZ1")
                                {
                                    QCZ1 = value_number;
                                }
                                if (identifier_str == "QDZ1")
                                {
                                    QDZ1 = value_number;
                                }
                                if (identifier_str == "QDZ2")
                                {
                                    QDZ2 = value_number;
                                }
                                if (identifier_str == "QDZ3")
                                {
                                    QDZ3 = value_number;
                                }
                                if (identifier_str == "QDZ4")
                                {
                                    QDZ4 = value_number;
                                }
                                if (identifier_str == "QDZ6")
                                {
                                    QDZ6 = value_number;
                                }
                                if (identifier_str == "QDZ7")
                                {
                                    QDZ7 = value_number;
                                }
                                if (identifier_str == "QDZ8")
                                {
                                    QDZ8 = value_number;
                                }
                                if (identifier_str == "QDZ9")
                                {
                                    QDZ9 = value_number;
                                }
                                if (identifier_str == "QEZ1")
                                {
                                    QEZ1 = value_number;
                                }
                                if (identifier_str == "QEZ2")
                                {
                                    QEZ2 = value_number;
                                }
                                if (identifier_str == "QEZ3")
                                {
                                    QEZ3 = value_number;
                                }
                                if (identifier_str == "QEZ4")
                                {
                                    QEZ4 = value_number;
                                }
                                if (identifier_str == "QEZ5")
                                {
                                    QEZ5 = value_number;
                                }
                                if (identifier_str == "QHZ1")
                                {
                                    QHZ1 = value_number;
                                }
                                if (identifier_str == "QHZ2")
                                {
                                    QHZ2 = value_number;
                                }
                                if (identifier_str == "QHZ3")
                                {
                                    QHZ3 = value_number;
                                }
                                if (identifier_str == "QHZ4")
                                {
                                    QHZ4 = value_number;
                                }
                                if (identifier_str == "SSZ1")
                                {
                                    SSZ1 = value_number;
                                }
                                if (identifier_str == "SSZ2")
                                {
                                    SSZ2 = value_number;
                                }
                                if (identifier_str == "SSZ3")
                                {
                                    SSZ3 = value_number;
                                }
                                if (identifier_str == "SSZ4")
                                {
                                    SSZ4 = value_number;
                                }
                                if (identifier_str == "QTZ1")
                                {
                                    QTZ1 = value_number;
                                }
                                if (identifier_str == "MBELT")
                                {
                                    MBELT = value_number;
                                }
                                if (identifier_str == "PECP1")
                                {
                                    PECP1 = value_number;
                                }
                                if (identifier_str == "PECP2")
                                {
                                    PECP2 = value_number;
                                }
                                if (identifier_str == "PDXP1")
                                {
                                    PDXP1 = value_number;
                                }
                                if (identifier_str == "PDXP2")
                                {
                                    PDXP2 = value_number;
                                }
                                if (identifier_str == "PDXP3")
                                {
                                    PDXP3 = value_number;
                                }
                                if (identifier_str == "PDYP1")
                                {
                                    PDYP1 = value_number;
                                }
                                if (identifier_str == "PDYP2")
                                {
                                    PDYP2 = value_number;
                                }
                                if (identifier_str == "PDYP3")
                                {
                                    PDYP3 = value_number;
                                }
                                if (identifier_str == "PDYP4")
                                {
                                    PDYP4 = value_number;
                                }
                                if (identifier_str == "PKYP1")
                                {
                                    PKYP1 = value_number;
                                }
                                if (identifier_str == "PHYP1")
                                {
                                    PHYP1 = value_number;
                                }
                                if (identifier_str == "PHYP2")
                                {
                                    PHYP2 = value_number;
                                }
                                if (identifier_str == "PHYP3")
                                {
                                    PHYP3 = value_number;
                                }
                                if (identifier_str == "PHYP4")
                                {
                                    PHYP4 = value_number;
                                }
                                if (identifier_str == "QDTP1")
                                {
                                    QDTP1 = value_number;
                                }
                                if (identifier_str == "QBRP1")
                                {
                                    QBRP1 = value_number;
                                }
                                if (identifier_str == "QCRP1")
                                {
                                    QCRP1 = value_number;
                                }
                                if (identifier_str == "QCRP2")
                                {
                                    QCRP2 = value_number;
                                }
                                if (identifier_str == "QDRP1")
                                {
                                    QDRP1 = value_number;
                                }
                                if (identifier_str == "QDRP2")
                                {
                                    QDRP2 = value_number;
                                }
                                if (identifier_str == "PA1")
                                {
                                    PA1 = value_number;
                                }
                                if (identifier_str == "PA2")
                                {
                                    PA2 = value_number;
                                }
                                if (identifier_str == "MC")
                                {
                                    MC = value_number;
                                }
                                if (identifier_str == "IC")
                                {
                                    IC = value_number;
                                }
                                if (identifier_str == "KX")
                                {
                                    KX = value_number;
                                }
                                if (identifier_str == "KY")
                                {
                                    KY = value_number;
                                }
                                if (identifier_str == "KP")
                                {
                                    KP = value_number;
                                }
                                if (identifier_str == "CX")
                                {
                                    CX = value_number;
                                }
                                if (identifier_str == "CY")
                                {
                                    CY = value_number;
                                }
                                if (identifier_str == "CP")
                                {
                                    CP = value_number;
                                }
                                if (identifier_str == "EP")
                                {
                                    EP = value_number;
                                }
                                if (identifier_str == "EP12")
                                {
                                    EP12 = value_number;
                                }
                                if (identifier_str == "BF2")
                                {
                                    BF2 = value_number;
                                }
                                if (identifier_str == "BP1")
                                {
                                    BP1 = value_number;
                                }
                                if (identifier_str == "BP2")
                                {
                                    BP2 = value_number;
                                }
                                if (identifier_str == "QV1")
                                {
                                    QV1 = value_number;
                                }
                                if (identifier_str == "QV2")
                                {
                                    QV2 = value_number;
                                }
                                if (identifier_str == "QFCX1")
                                {
                                    QFCX1 = value_number;
                                }
                                if (identifier_str == "QFCY1")
                                {
                                    QFCY1 = value_number;
                                }
                                if (identifier_str == "QFCG1")
                                {
                                    QFCG1 = value_number;
                                }
                                if (identifier_str == "QFZ1")
                                {
                                    QFZ1 = value_number;
                                }
                                if (identifier_str == "QFZ2")
                                {
                                    QFZ2 = value_number;
                                }
                            }
                        }
                    }
                }
            }

            virtual real_T GetNoLoadOuterDiameter() const override
            {
                /*TODO: Is this correct?*/
                return UNLOADED_RADIUS;
            }

            virtual real_T GetDryRollingFrictionCoefficient() const override
            {
                /*TODO: Is this correct?*/
                /*Probably not, Judging by magnitude, this is probably the sliding coefficient.*/
                /*No idea which is correct, though ¯\_(ツ)_/¯ */
                return PDX1;
            }

            virtual real_T GetAngularInertia() const override
            {
                /*TODO: is this correct?*/
                return IC;
            }

            real_T UNLOADED_RADIUS = static_cast<real_T>(0.0f);
            real_T WIDTH = static_cast<real_T>(0.0f);
            real_T ASPECT_RATIO = static_cast<real_T>(0.0f);
            real_T RIM_RADIUS = static_cast<real_T>(0.0f);
            real_T RIM_WIDTH = static_cast<real_T>(0.0f);
            real_T VERTICAL_STIFFNESS = static_cast<real_T>(0.0f);
            real_T VERTICAL_DAMPING = static_cast<real_T>(0.0f);
            real_T BREFF = static_cast<real_T>(0.0f);
            real_T DREFF = static_cast<real_T>(0.0f);
            real_T FREFF = static_cast<real_T>(0.0f);
            real_T FNOMIN = static_cast<real_T>(0.0f);
            real_T KPUMIN = static_cast<real_T>(0.0f);
            real_T KPUMAX = static_cast<real_T>(0.0f);
            real_T ALPMIN = static_cast<real_T>(0.0f);
            real_T ALPMAX = static_cast<real_T>(0.0f);
            real_T CAMMIN = static_cast<real_T>(0.0f);
            real_T CAMMAX = static_cast<real_T>(0.0f);
            real_T FZMIN = static_cast<real_T>(0.0f);
            real_T FZMAX = static_cast<real_T>(0.0f);
            real_T LFZO = static_cast<real_T>(0.0f);
            real_T LCX = static_cast<real_T>(0.0f);
            real_T LMUX = static_cast<real_T>(0.0f);
            real_T LEX = static_cast<real_T>(0.0f);
            real_T LKX = static_cast<real_T>(0.0f);
            real_T LHX = static_cast<real_T>(0.0f);
            real_T LVX = static_cast<real_T>(0.0f);
            real_T LGAX = static_cast<real_T>(0.0f);
            real_T LCY = static_cast<real_T>(0.0f);
            real_T LMUY = static_cast<real_T>(0.0f);
            real_T LEY = static_cast<real_T>(0.0f);
            real_T LKY = static_cast<real_T>(0.0f);
            real_T LHY = static_cast<real_T>(0.0f);
            real_T LVY = static_cast<real_T>(0.0f);
            real_T LGAY = static_cast<real_T>(0.0f);
            real_T LTR = static_cast<real_T>(0.0f);
            real_T LRES = static_cast<real_T>(0.0f);
            real_T LGAZ = static_cast<real_T>(0.0f);
            real_T LXAL = static_cast<real_T>(0.0f);
            real_T LYKA = static_cast<real_T>(0.0f);
            real_T LVYKA = static_cast<real_T>(0.0f);
            real_T LS = static_cast<real_T>(0.0f);
            real_T LSGKP = static_cast<real_T>(0.0f);
            real_T LSGAL = static_cast<real_T>(0.0f);
            real_T LGYR = static_cast<real_T>(0.0f);
            real_T LMX = static_cast<real_T>(0.0f);
            real_T LVMX = static_cast<real_T>(0.0f);
            real_T LMY = static_cast<real_T>(0.0f);
            real_T PCX1 = static_cast<real_T>(0.0f);
            real_T PDX1 = static_cast<real_T>(0.0f);
            real_T PDX2 = static_cast<real_T>(0.0f);
            real_T PDX3 = static_cast<real_T>(0.0f);
            real_T PEX1 = static_cast<real_T>(0.0f);
            real_T PEX2 = static_cast<real_T>(0.0f);
            real_T PEX3 = static_cast<real_T>(0.0f);
            real_T PEX4 = static_cast<real_T>(0.0f);
            real_T PKX1 = static_cast<real_T>(0.0f);
            real_T PKX2 = static_cast<real_T>(0.0f);
            real_T PKX3 = static_cast<real_T>(0.0f);
            real_T PHX1 = static_cast<real_T>(0.0f);
            real_T PHX2 = static_cast<real_T>(0.0f);
            real_T PVX1 = static_cast<real_T>(0.0f);
            real_T PVX2 = static_cast<real_T>(0.0f);
            real_T RBX1 = static_cast<real_T>(0.0f);
            real_T RBX2 = static_cast<real_T>(0.0f);
            real_T RCX1 = static_cast<real_T>(0.0f);
            real_T REX1 = static_cast<real_T>(0.0f);
            real_T REX2 = static_cast<real_T>(0.0f);
            real_T RHX1 = static_cast<real_T>(0.0f);
            real_T PTX1 = static_cast<real_T>(0.0f);
            real_T PTX2 = static_cast<real_T>(0.0f);
            real_T PTX3 = static_cast<real_T>(0.0f);
            real_T QSX1 = static_cast<real_T>(0.0f);
            real_T QSX2 = static_cast<real_T>(0.0f);
            real_T QSX3 = static_cast<real_T>(0.0f);
            real_T PCY1 = static_cast<real_T>(0.0f);
            real_T PDY1 = static_cast<real_T>(0.0f);
            real_T PDY2 = static_cast<real_T>(0.0f);
            real_T PDY3 = static_cast<real_T>(0.0f);
            real_T PEY1 = static_cast<real_T>(0.0f);
            real_T PEY2 = static_cast<real_T>(0.0f);
            real_T PEY3 = static_cast<real_T>(0.0f);
            real_T PEY4 = static_cast<real_T>(0.0f);
            real_T PKY1 = static_cast<real_T>(0.0f);
            real_T PKY2 = static_cast<real_T>(0.0f);
            real_T PKY3 = static_cast<real_T>(0.0f);
            real_T PHY1 = static_cast<real_T>(0.0f);
            real_T PHY2 = static_cast<real_T>(0.0f);
            real_T PHY3 = static_cast<real_T>(0.0f);
            real_T PVY1 = static_cast<real_T>(0.0f);
            real_T PVY2 = static_cast<real_T>(0.0f);
            real_T PVY3 = static_cast<real_T>(0.0f);
            real_T PVY4 = static_cast<real_T>(0.0f);
            real_T RBY1 = static_cast<real_T>(0.0f);
            real_T RBY2 = static_cast<real_T>(0.0f);
            real_T RBY3 = static_cast<real_T>(0.0f);
            real_T RCY1 = static_cast<real_T>(0.0f);
            real_T REY1 = static_cast<real_T>(0.0f);
            real_T REY2 = static_cast<real_T>(0.0f);
            real_T RHY1 = static_cast<real_T>(0.0f);
            real_T RHY2 = static_cast<real_T>(0.0f);
            real_T RVY1 = static_cast<real_T>(0.0f);
            real_T RVY2 = static_cast<real_T>(0.0f);
            real_T RVY3 = static_cast<real_T>(0.0f);
            real_T RVY4 = static_cast<real_T>(0.0f);
            real_T RVY5 = static_cast<real_T>(0.0f);
            real_T RVY6 = static_cast<real_T>(0.0f);
            real_T PTY1 = static_cast<real_T>(0.0f);
            real_T PTY2 = static_cast<real_T>(0.0f);
            real_T QSY1 = static_cast<real_T>(0.0f);
            real_T QSY2 = static_cast<real_T>(0.0f);
            real_T QSY3 = static_cast<real_T>(0.0f);
            real_T QSY4 = static_cast<real_T>(0.0f);
            real_T QBZ1 = static_cast<real_T>(0.0f);
            real_T QBZ2 = static_cast<real_T>(0.0f);
            real_T QBZ3 = static_cast<real_T>(0.0f);
            real_T QBZ4 = static_cast<real_T>(0.0f);
            real_T QBZ5 = static_cast<real_T>(0.0f);
            real_T QBZ9 = static_cast<real_T>(0.0f);
            real_T QBZ10 = static_cast<real_T>(0.0f);
            real_T QCZ1 = static_cast<real_T>(0.0f);
            real_T QDZ1 = static_cast<real_T>(0.0f);
            real_T QDZ2 = static_cast<real_T>(0.0f);
            real_T QDZ3 = static_cast<real_T>(0.0f);
            real_T QDZ4 = static_cast<real_T>(0.0f);
            real_T QDZ6 = static_cast<real_T>(0.0f);
            real_T QDZ7 = static_cast<real_T>(0.0f);
            real_T QDZ8 = static_cast<real_T>(0.0f);
            real_T QDZ9 = static_cast<real_T>(0.0f);
            real_T QEZ1 = static_cast<real_T>(0.0f);
            real_T QEZ2 = static_cast<real_T>(0.0f);
            real_T QEZ3 = static_cast<real_T>(0.0f);
            real_T QEZ4 = static_cast<real_T>(0.0f);
            real_T QEZ5 = static_cast<real_T>(0.0f);
            real_T QHZ1 = static_cast<real_T>(0.0f);
            real_T QHZ2 = static_cast<real_T>(0.0f);
            real_T QHZ3 = static_cast<real_T>(0.0f);
            real_T QHZ4 = static_cast<real_T>(0.0f);
            real_T SSZ1 = static_cast<real_T>(0.0f);
            real_T SSZ2 = static_cast<real_T>(0.0f);
            real_T SSZ3 = static_cast<real_T>(0.0f);
            real_T SSZ4 = static_cast<real_T>(0.0f);
            real_T QTZ1 = static_cast<real_T>(0.0f);
            real_T MBELT = static_cast<real_T>(0.0f);
            real_T PECP1 = static_cast<real_T>(0.0f);
            real_T PECP2 = static_cast<real_T>(0.0f);
            real_T PDXP1 = static_cast<real_T>(0.0f);
            real_T PDXP2 = static_cast<real_T>(0.0f);
            real_T PDXP3 = static_cast<real_T>(0.0f);
            real_T PDYP1 = static_cast<real_T>(0.0f);
            real_T PDYP2 = static_cast<real_T>(0.0f);
            real_T PDYP3 = static_cast<real_T>(0.0f);
            real_T PDYP4 = static_cast<real_T>(0.0f);
            real_T PKYP1 = static_cast<real_T>(0.0f);
            real_T PHYP1 = static_cast<real_T>(0.0f);
            real_T PHYP2 = static_cast<real_T>(0.0f);
            real_T PHYP3 = static_cast<real_T>(0.0f);
            real_T PHYP4 = static_cast<real_T>(0.0f);
            real_T QDTP1 = static_cast<real_T>(0.0f);
            real_T QBRP1 = static_cast<real_T>(0.0f);
            real_T QCRP1 = static_cast<real_T>(0.0f);
            real_T QCRP2 = static_cast<real_T>(0.0f);
            real_T QDRP1 = static_cast<real_T>(0.0f);
            real_T QDRP2 = static_cast<real_T>(0.0f);
            real_T PA1 = static_cast<real_T>(0.0f);
            real_T PA2 = static_cast<real_T>(0.0f);
            real_T MC = static_cast<real_T>(0.0f);
            real_T IC = static_cast<real_T>(0.0f);
            real_T KX = static_cast<real_T>(0.0f);
            real_T KY = static_cast<real_T>(0.0f);
            real_T KP = static_cast<real_T>(0.0f);
            real_T CX = static_cast<real_T>(0.0f);
            real_T CY = static_cast<real_T>(0.0f);
            real_T CP = static_cast<real_T>(0.0f);
            real_T EP = static_cast<real_T>(0.0f);
            real_T EP12 = static_cast<real_T>(0.0f);
            real_T BF2 = static_cast<real_T>(0.0f);
            real_T BP1 = static_cast<real_T>(0.0f);
            real_T BP2 = static_cast<real_T>(0.0f);
            real_T QV1 = static_cast<real_T>(0.0f);
            real_T QV2 = static_cast<real_T>(0.0f);
            real_T QFCX1 = static_cast<real_T>(0.0f);
            real_T QFCY1 = static_cast<real_T>(0.0f);
            real_T QFCG1 = static_cast<real_T>(0.0f);
            real_T QFZ1 = static_cast<real_T>(0.0f);
            real_T QFZ2 = static_cast<real_T>(0.0f);

        private:
            std::map<std::string, real_T> _parameters;
    };
}}

#endif
