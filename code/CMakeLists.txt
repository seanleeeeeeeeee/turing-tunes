idf_component_register(
    SRCS "main.cpp" "stick.cpp" "controls.cpp" "lcd.cpp" "persist.cpp" "renderer.cpp" "simulation.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2s esp_driver_i2c esp_adc esp_lcd nvs_flash
)