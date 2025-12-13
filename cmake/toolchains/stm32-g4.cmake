set(TARGET_FLAGS
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
)

add_compile_options(${TARGET_FLAGS})
add_link_options(${TARGET_FLAGS})

add_compile_options(
  ${MCPU_FLAGS}
  -ffunction-sections
  -fdata-sections
)

add_link_options(
  ${MCPU_FLAGS}
  -Wl,--gc-sections
)

add_compile_options(
  $<$<COMPILE_LANGUAGE:ASM>:-mthumb>
)
