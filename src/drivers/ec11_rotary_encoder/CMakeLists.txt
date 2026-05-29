############################################################################
# drivers/ec11_rotary_encoder/CMakeLists.txt
#
# PX4 build definition for the EC11 rotary encoder driver.
############################################################################

px4_add_module(
    MODULE drivers__ec11_rotary_encoder
    MAIN   ec11_rotary_encoder
    SRCS
        EC11RotaryEncoder.cpp
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml
)
