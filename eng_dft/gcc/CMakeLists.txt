#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.21)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(eng_dft C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# for use printf & scanf with float
set(COMMON_FLAGS "-specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float")

#Uncomment for hardware floating point

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(
    ${CMAKE_SOURCE_DIR}../../driver/inc
    ${CMAKE_SOURCE_DIR}../source
    ${CMAKE_SOURCE_DIR}../../mcu/common
    ${CMAKE_SOURCE_DIR}/include
)

set(APP_FILES
${CMAKE_SOURCE_DIR}../source/*.c
${CMAKE_SOURCE_DIR}../../driver/src/*.c
${CMAKE_SOURCE_DIR}../EWARM/startup_hc32l07x.s
${CMAKE_SOURCE_DIR}../../common/*.c
)


set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/hc32l07x_flash.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/eng_dft.map)
add_link_options(-T ${LINKER_SCRIPT})

#link_directories("Drivers/CMSIS/Lib")
#link_libraries("arm_cortexM4lf_math.a")

add_executable(eng_dft.elf ${APP_FILES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/eng_dft.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/eng_dft.bin)

add_custom_command(TARGET eng_dft.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:eng_dft.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:eng_dft.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
