idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    idf_component_register(INCLUDE_DIRS include)
else()
    idf_component_register(SRCS "src/ymodem.c"
                           INCLUDE_DIRS include
                           PRIV_REQUIRES driver)
endif()