# ESP32 MCP2515 CAN Controller Library
# CMakeLists.txt for ESP-IDF

idf_component_register(
    SRCS "mcp2515.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver
)

# Set component C++ standard
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_11)

# Add compile definitions
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    ESP32
)

# Optimization flags
if(CONFIG_MCP2515_OPTIMIZE_SPEED)
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -O2
        -ffast-math
    )
endif()

# Enable warnings
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

message(STATUS "ESP32-MCP2515 CAN Library configured")
message(STATUS "  - SPI Host: ${CONFIG_MCP2515_SPI_HOST}")
message(STATUS "  - Use Interrupts: ${CONFIG_MCP2515_USE_INTERRUPTS}")
message(STATUS "  - Use Mutex: ${CONFIG_MCP2515_USE_MUTEX}")
