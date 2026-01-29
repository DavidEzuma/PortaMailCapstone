#!/bin/bash

# Define the path to the Teensy platform.txt file
PLATFORM_FILE="$HOME/.arduino15/packages/teensy/hardware/avr/1.59.0/platform.txt"

echo "Checking $PLATFORM_FILE..."

if [ ! -f "$PLATFORM_FILE" ]; then
    echo "Error: platform.txt not found! Did you install the teensy:avr core?"
    exit 1
fi

# We need to add {compiler.libraries.ldflags} to the linker command.
# This variable contains the path to the precompiled Micro-ROS library.

# Backup
cp "$PLATFORM_FILE" "$PLATFORM_FILE.bak"

echo "Patching platform.txt..."

# 1. Patch the C linker recipe
sed -i 's/recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags}/recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags} {compiler.libraries.ldflags}/' "$PLATFORM_FILE"

# 2. Patch the CPP linker recipe (if separate)
sed -i 's/recipe.cpp.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags}/recipe.cpp.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags} {compiler.libraries.ldflags}/' "$PLATFORM_FILE"

# 3. CRITICAL: Add the variable definition itself if missing
# Some versions of arduino-cli freak out if the variable isn't defined at all in the file
if ! grep -q "compiler.libraries.ldflags=" "$PLATFORM_FILE"; then
    echo "" >> "$PLATFORM_FILE"
    echo "# Micro-ROS Patch" >> "$PLATFORM_FILE"
    echo "compiler.libraries.ldflags=" >> "$PLATFORM_FILE"
fi

echo "Patch applied successfully."