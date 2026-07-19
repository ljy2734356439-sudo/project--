# main/CMakeLists.txt

idf_component_register(
    SRCS
        "main.c"
        "shared_data.c"
        "sensor_task.c"
        "audio_task.c"
        "display_task.c"
        "elevator_state_machine.c"
        "subjective_score.c"
        "score_task.c"
        "button_task.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_lcd
        driver
        esp_driver_i2c
        esp_driver_i2s
        freertos
)
