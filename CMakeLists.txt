idf_component_register(
  SRCS "src/tef/lora.cpp" "src/sx1262.cpp" "src/sx1276.cpp"
  PRIV_REQUIRES driver esp_driver_spi esp_driver_gpio
  INCLUDE_DIRS "include"
)
