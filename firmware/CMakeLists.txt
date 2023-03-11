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
		   srcs/usb_descriptors.c
	       )

target_include_directories(pigeki PRIVATE ${CMAKE_CURRENT_LIST_DIR}/srcs)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pigeki pico_stdlib
							 pico_multicore
							 pico_unique_id
							 tinyusb_device
							 tinyusb_board
							 hardware_pwm
							 hardware_adc
							 hardware_dma)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pigeki)
