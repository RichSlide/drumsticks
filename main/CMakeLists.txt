set(srcs "main.c"
         "gatt_svr.c")

idf_component_register(SRCS "IMU.c" "${srcs}"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash
                                     bt
                                     console
                                     esp_driver_i2c)
