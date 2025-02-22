Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
        "$BUILD_DIR/${PROGNAME}.elf", "$PROJECT_DIR/output/${PROGNAME}.hex"
    ]), "Building $PROJECT_DIR/output/${PROGNAME}.hex")
)

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "binary", "-R", ".eeprom",
        "$BUILD_DIR/${PROGNAME}.elf", "$PROJECT_DIR/output/${PROGNAME}.bin"
    ]), "Building $PROJECT_DIR/output/${PROGNAME}.bin")
)

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-j", ".eeprom", "-O", "ihex",
        "$BUILD_DIR/${PROGNAME}.elf", "$PROJECT_DIR/output/${PROGNAME}.eep"
    ]), "Building EEPROM file $PROJECT_DIR/output/${PROGNAME}.eep")
)

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-j", ".eeprom", "-O", "binary",
        "$BUILD_DIR/${PROGNAME}.elf", "$PROJECT_DIR/output/${PROGNAME}.eep.bin"
    ]), "Building EEPROM file $PROJECT_DIR/output/${PROGNAME}.eep.bin")
)