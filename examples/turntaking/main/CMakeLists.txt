idf_component_register(SRCS "app/main.cpp" "app/utils.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES tef-boards tef-lora nvs_flash)
