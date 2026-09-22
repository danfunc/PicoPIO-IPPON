# ==============================================================================
# CheckBinarySize.cmake
# Validates that generated binary (.bin) does not exceed the designated limit.
# Parameters passed via -D:
#   BINARY_FILE : Full path to .bin file
#   MAX_BYTES   : Maximum allowed size in bytes (e.g. 1048576 for 1MB)
#   TARGET_NAME : Name of the target for reporting
# ==============================================================================

if(NOT DEFINED BINARY_FILE)
    message(FATAL_ERROR "CheckBinarySize.cmake: BINARY_FILE not defined")
endif()

if(NOT DEFINED MAX_BYTES)
    set(MAX_BYTES 1048576) # Default 1MB
endif()

if(NOT DEFINED TARGET_NAME)
    set(TARGET_NAME "binary")
endif()

if(EXISTS "${BINARY_FILE}")
    file(SIZE "${BINARY_FILE}" BIN_SIZE)
    if(BIN_SIZE GREATER MAX_BYTES)
        math(EXPR OVERFLOW "${BIN_SIZE} - ${MAX_BYTES}")
        message(FATAL_ERROR
            "\n[SIZE CHECK FAILED] Target '${TARGET_NAME}' binary size ${BIN_SIZE} bytes "
            "exceeds 1MB firmware limit of ${MAX_BYTES} bytes by ${OVERFLOW} bytes! "
            "Flash staging area is at risk of corruption."
        )
    else()
        math(EXPR REMAINING "${MAX_BYTES} - ${BIN_SIZE}")
        math(EXPR PCT "(${BIN_SIZE} * 100) / ${MAX_BYTES}")
        message(STATUS
            "[SIZE CHECK PASSED] ${TARGET_NAME}.bin: ${BIN_SIZE} bytes / ${MAX_BYTES} bytes max "
            "(${PCT}% utilized, ${REMAINING} bytes headroom within 1MB firmware slot)"
        )
    endif()
else()
    message(WARNING "CheckBinarySize.cmake: Binary file ${BINARY_FILE} not found; skipping size check.")
endif()
