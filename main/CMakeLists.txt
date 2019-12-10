set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_SRCS
        "src/app_main.c"
        "src/utils.c"
        "src/wifi.c"
        "src/mqtt.c"
        "src/pcf8574.c")

register_component()
