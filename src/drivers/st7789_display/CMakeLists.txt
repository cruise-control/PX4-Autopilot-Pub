############################################################################
# drivers/st7789_display/CMakeLists.txt
############################################################################

px4_add_module(
    MODULE drivers__st7789_display
    MAIN   st7789_display
    SRCS
        ST7789Display.cpp
    DEPENDS
        px4_work_queue
    MODULE_CONFIG
        module.yaml
)
