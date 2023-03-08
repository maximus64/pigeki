cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(pigeki)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
add_executable(pigeki
	       srcs/main.c
	       )

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pigeki pico_stdlib pico_multicore)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pigeki)