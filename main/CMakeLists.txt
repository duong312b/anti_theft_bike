idf_component_register(SRCS "main.c" "motion.c" "display.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_gpio esp_timer
                                mpu6050 lcd_i2c gnss modem)
