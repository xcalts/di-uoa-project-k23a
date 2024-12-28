#ifndef CONSTANTS_H
#define CONSTANTS_H

/**********************/
/* Standard Libraries */
/**********************/
#include <string>

/************/
/* Commands */
/************/
const std::string CLI_INIT = "initialize";
const std::string CLI_EVAL = "evaluate";
const std::string CLI_GTNN = "generate-groundtruth";

/**************/
/* Algorithms */
/**************/
const std::string VAMANA = "vamana";
const std::string FILTERED_VAMANA = "filtered-vamana";
const std::string STICHED_VAMANA = "stiched-vamana";

/*******/
/* Log */
/*******/
const std::string INFO = "🈯 [info] ";
const std::string COMMAND = "🈁 [command] ";
const std::string SUCCESS = "  🈯 [success] ";
const std::string DEBUGG = "  🈚 [debug] ";
const std::string EXCEPTION = "🉐 [exception] ";

/**********/
/* Extras */
/**********/
const std::string WELCOMING_MSG = R"(
+---------------------------------------------------------------------------------+
|  _    _      _                            _          _   __ _____  _____        |
| | |  | |    | |                          | |        | | / // __  \|____ |       |
| | |  | | ___| | ___ ___  _ __ ___   ___  | |_ ___   | |/ / `' / /'    / / __ _  |
| | |/\| |/ _ \ |/ __/ _ \| '_ ` _ \ / _ \ | __/ _ \  |    \   / /      \ \/ _` | |
| \  /\  /  __/ | (_| (_) | | | | | |  __/ | || (_) | | |\  \./ /___.___/ / (_| | |
|  \/  \/ \___|_|\___\___/|_| |_| |_|\___|  \__\___/  \_| \_/\_____/\____/ \__,_| |
|                                                                                 | 
|                                         by Christos Kaltsas & Natalia Krikkeli  | 
+---------------------------------------------------------------------------------+                                                                               
)";

#endif // CONSTANTS_H