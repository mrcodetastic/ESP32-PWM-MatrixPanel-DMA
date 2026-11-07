idf_component_register(
    SRCS "../test/bitscrambler/main/main.c"
    PRIV_REQUIRES esp_driver_bitscrambler
    INCLUDE_DIRS "."
)

target_bitscrambler_add_src("../test/bitscrambler/main/nibble_swap.bsasm")