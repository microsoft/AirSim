#ifndef air_brushwheelparameters_hpp
#define air_brushwheelparameters_hpp

#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include "automobile/WheelParameters.hpp"
#include "common/Common.hpp"

namespace msr { namespace airlib {

	class BrushWheelParameters : public WheelParameters
	{
		public:
			BrushWheelParameters() {};

			BrushWheelParameters(std::string inputFile)
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
								if (identifier_str == "ANGULARINERTIA")
								{
									ANGULARINERTIA = value_number;
								}
								if (identifier_str == "AIRPRESSURE")
								{
									AIRPRESSURE = value_number;
								}
								if (identifier_str == "RIMDIAMETER")
								{
									RIMDIAMETER = value_number;
								}
								if (identifier_str == "SIDEWALLDEFLECTION")
								{
									SIDEWALLDEFLECTION = value_number;
								}
								if (identifier_str == "ASPECTRATIO")
								{
									ASPECTRATIO = value_number;
								}
								if (identifier_str == "RIMWIDTH")
								{
									RIMWIDTH = value_number;
								}
								if (identifier_str == "TIREWIDTH")
								{
									TIREWIDTH = value_number;
								}
								if (identifier_str == "LOADINDEX")
								{
									if (value_number == static_cast<real_T>(62))
									{
										LOADINDEX = static_cast<real_T>(265);
									}
									if (value_number == static_cast<real_T>(63))
									{
										LOADINDEX = static_cast<real_T>(272);
									}
									if (value_number == static_cast<real_T>(64))
									{
										LOADINDEX = static_cast<real_T>(280);
									}
									if (value_number == static_cast<real_T>(65))
									{
										LOADINDEX = static_cast<real_T>(290);
									}
									if (value_number == static_cast<real_T>(66))
									{
										LOADINDEX = static_cast<real_T>(300);
									}
									if (value_number == static_cast<real_T>(67))
									{
										LOADINDEX = static_cast<real_T>(307);
									}
									if (value_number == static_cast<real_T>(68))
									{
										LOADINDEX = static_cast<real_T>(315);
									}
									if (value_number == static_cast<real_T>(69))
									{
										LOADINDEX = static_cast<real_T>(325);
									}
									if (value_number == static_cast<real_T>(70))
									{
										LOADINDEX = static_cast<real_T>(335);
									}
									if (value_number == static_cast<real_T>(71))
									{
										LOADINDEX = static_cast<real_T>(345);
									}
									if (value_number == static_cast<real_T>(72))
									{
										LOADINDEX = static_cast<real_T>(355);
									}
									if (value_number == static_cast<real_T>(73))
									{
										LOADINDEX = static_cast<real_T>(365);
									}
									if (value_number == static_cast<real_T>(74))
									{
										LOADINDEX = static_cast<real_T>(375);
									}
									if (value_number == static_cast<real_T>(75))
									{
										LOADINDEX = static_cast<real_T>(387);
									}
									if (value_number == static_cast<real_T>(76))
									{
										LOADINDEX = static_cast<real_T>(400);
									}
									if (value_number == static_cast<real_T>(83))
									{
										LOADINDEX = static_cast<real_T>(487);
									}
									if (value_number == static_cast<real_T>(84))
									{
										LOADINDEX = static_cast<real_T>(500);
									}
									if (value_number == static_cast<real_T>(85))
									{
										LOADINDEX = static_cast<real_T>(515);
									}
									if (value_number == static_cast<real_T>(86))
									{
										LOADINDEX = static_cast<real_T>(530);
									}
									if (value_number == static_cast<real_T>(88))
									{
										LOADINDEX = static_cast<real_T>(560);
									}
									if (value_number == static_cast<real_T>(89))
									{
										LOADINDEX = static_cast<real_T>(580);
									}
									if (value_number == static_cast<real_T>(90))
									{
										LOADINDEX = static_cast<real_T>(600);
									}
									if (value_number == static_cast<real_T>(91))
									{
										LOADINDEX = static_cast<real_T>(615);
									}
									if (value_number == static_cast<real_T>(93))
									{
										LOADINDEX = static_cast<real_T>(650);
									}
									if (value_number == static_cast<real_T>(94))
									{
										LOADINDEX = static_cast<real_T>(670);
									}
									if (value_number == static_cast<real_T>(95))
									{
										LOADINDEX = static_cast<real_T>(690);
									}
									if (value_number == static_cast<real_T>(96))
									{
										LOADINDEX = static_cast<real_T>(710);
									}
									if (value_number == static_cast<real_T>(97))
									{
										LOADINDEX = static_cast<real_T>(730);
									}
									if (value_number == static_cast<real_T>(104))
									{
										LOADINDEX = static_cast<real_T>(900);
									}
									if (value_number == static_cast<real_T>(105))
									{
										LOADINDEX = static_cast<real_T>(925);
									}
									if (value_number == static_cast<real_T>(106))
									{
										LOADINDEX = static_cast<real_T>(950);
									}
									if (value_number == static_cast<real_T>(107))
									{
										LOADINDEX = static_cast<real_T>(975);
									}
									if (value_number == static_cast<real_T>(108))
									{
										LOADINDEX = static_cast<real_T>(1000);
									}
								}
								if (identifier_str == "LATERALSTIFFNESSPERMETER")
								{
									LATERALSTIFFNESSPERMETER = value_number;
								}
								if (identifier_str == "DRYROLLINGFRICTIONCOEFFICIENT")
								{
									DRYROLLINGFRICTIONCOEFFICIENT = value_number;
								}
								if (identifier_str == "DRYSLIDINGFRICTIONCOEFFICIENT")
								{
									DRYSLIDINGFRICTIONCOEFFICIENT = value_number;
								}
							}
						}
					}
				}
			}

			virtual real_T GetNoLoadOuterDiameter() const override
			{
				real_T drim;

				if (ASPECTRATIO >= 75.0f)
					drim = TIREWIDTH * 0.7f / 25.4f;
				else if (ASPECTRATIO >= 60.0f)
					drim = TIREWIDTH * 0.75f / 25.4f;
				else if (ASPECTRATIO >= 50.0f)
					drim = TIREWIDTH * 0.8f / 25.4f;
				else if (ASPECTRATIO >= 45.0f)
					drim = TIREWIDTH * 0.85f / 25.4f;
				else if (ASPECTRATIO >= 30.0f)
					drim = TIREWIDTH * 0.9f / 25.4f;
				else
					drim = TIREWIDTH * 0.92f / 25.4f;

				/*Compare this with the actual rim width used*/
				real_T drimck = drim - RIMWIDTH;

				/*Sidewall height*/
				real_T h1 = TIREWIDTH * ASPECTRATIO / static_cast<real_T>(100.0);

				/*Sidewall width changes 2.5mm for every 0.5" change in rim width*/
				real_T secht = h1 + (2.5f * drimck); //TODO: units?

				/*Rim diameter*/
				real_T dr = RIMDIAMETER * 25.4f;

				/*Outer diameter with no load*/
				real_T d = (2 * secht) + dr;

				return d / 1000.0f;
			}

			virtual real_T GetAngularInertia() const override
			{
				return ANGULARINERTIA;
			}

			virtual real_T GetDryRollingFrictionCoefficient() const override
			{
				return DRYROLLINGFRICTIONCOEFFICIENT;
			}

			real_T ANGULARINERTIA = static_cast<real_T>(0.0f); //Jz

			/*For contact patch*/
			real_T AIRPRESSURE = static_cast<real_T>(0.0f); //apres 
			real_T RIMDIAMETER = static_cast<real_T>(0.0f); //dr 
			real_T SIDEWALLDEFLECTION = static_cast<real_T>(0.0f); //fr 
			real_T ASPECTRATIO = static_cast<real_T>(0.0f); //aspectratio 
			real_T RIMWIDTH = static_cast<real_T>(0.0f); //rimwidth 
			real_T TIREWIDTH = static_cast<real_T>(0.0f); //tirewidth 
			real_T LOADINDEX = static_cast<real_T>(0.0f); //loadindex 

			real_T DRYROLLINGFRICTIONCOEFFICIENT = static_cast<real_T>(0.0f);
			real_T DRYSLIDINGFRICTIONCOEFFICIENT = static_cast<real_T>(0.0f);
			
			real_T LATERALSTIFFNESSPERMETER = static_cast<real_T>(0.0f);
			
	};

}}

#endif