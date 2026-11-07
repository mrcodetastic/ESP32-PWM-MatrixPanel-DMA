idf_component_register(
    SRCS "main.c"
    PRIV_REQUIRES esp_driver_bitscrambler
    INCLUDE_DIRS "."
)

target_bitscrambler_add_src("nibble_swap.bsasm")