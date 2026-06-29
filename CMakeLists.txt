cmake_minimum_required(VERSION 3.15)
project(DroneValidationEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- NEW: Automatically download the nlohmann JSON library ---
include(FetchContent)
FetchContent_Declare(json
  URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
)
FetchContent_MakeAvailable(json)
# -----------------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCE_FILES
    src/main.cpp
    src/module1_feasibility/FeasibilityGate.cpp
    src/module2_kinematics/MissionProfiler.cpp
    src/module3_physics/MassIteration.cpp
    src/module3_physics/Aerodynamics.cpp
    src/module3_physics/Thermodynamics.cpp
    src/module3_physics/Structures.cpp
    src/module4_optimizer/DiscreteMINLP.cpp
    src/module5_validation/SensitivityAnalysis.cpp
    src/utils/JsonParser.cpp
)

add_executable(DroneEngine ${SOURCE_FILES})
target_compile_definitions(DroneEngine PRIVATE _USE_MATH_DEFINES)

# Link the JSON library to our executable
target_link_libraries(DroneEngine PRIVATE nlohmann_json::nlohmann_json)