#!/bin/bash

# Define the path to the Teensy platform.txt file
# This is where arduino-cli installs the Teensy core
PLATFORM_FILE="$HOME/.arduino15/packages/teensy/hardware/avr/1.59.0/platform.txt"

echo "Checking $PLATFORM_FILE..."

if [ ! -f "$PLATFORM_FILE" ]; then
    echo "Error: platform.txt not found! Did you install the teensy:avr core?"
    exit 1
fi

# Check if already patched
if grep -q "compiler.libraries.ldflags" "$PLATFORM_FILE"; then
    echo "Success: platform.txt is already patched."
else
    echo "Patching platform.txt..."
    
    # Backup the original file
    cp "$PLATFORM_FILE" "$PLATFORM_FILE.bak"
    
    # Append the {compiler.libraries.ldflags} variable to the linker recipe
    # We add it right after the ELF flags
    sed -i 's/recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags}/recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {compiler.c.elf.flags} {compiler.libraries.ldflags}/' "$PLATFORM_FILE"
    
    echo "Patch applied."
fi
