idf_component_register(SRCS "music_task.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES nvs_flash audio_pipeline audio_stream esp_peripherals input_key_service playlist)

message([MUSIC_TASK] " Building")
