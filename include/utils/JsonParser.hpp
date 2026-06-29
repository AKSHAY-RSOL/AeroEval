#ifndef JSON_PARSER_HPP
#define JSON_PARSER_HPP

#include "DataStructures.hpp"
#include <string>

namespace Utils {

    // JSON parsing utility wrapping nlohmann/json.
    class JsonParser {
    public:
        // Load user input sizing parameters.
        static EngineData::UserInput parseUserInput(const std::string& filepath);

        // Load parameter constraints and optimization locks.
        static EngineData::UserLocks parseUserLocks(const std::string& filepath);

        // Retrieve battery chemistry profile from profiles JSON.
        static EngineData::BatteryChemistry parseChemistry(const std::string& filepath, const std::string& chem_name);

        // Load discrete options for COTS propellers and motor KV ratings.
        static EngineData::COTS_Sets parseCOTSDatabase(const std::string& filepath);

        // Load ESC parasitics and ON-state resistance values.
        static EngineData::ESCHardware parseESCHardware(const std::string& filepath);
    };

}

#endif // JSON_PARSER_HPP