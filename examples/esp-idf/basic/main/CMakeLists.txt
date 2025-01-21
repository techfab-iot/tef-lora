set(component_srcs)

idf_component_register(SRCS "main.cpp" "${component_srcs}"
                       INCLUDE_DIRS ".")


