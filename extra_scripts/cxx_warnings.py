Import("env")
# Suppress warnings that only fire in Teensy framework C++ headers.
# These flags are invalid for C, so they cannot go in build_flags.
env.Append(CXXFLAGS=[
    "-Wno-deprecated-copy",    # DMAChannel.h copy-assignment deprecation
    "-Wno-ignored-qualifiers", # EEPROM.h const return type qualifiers
])
