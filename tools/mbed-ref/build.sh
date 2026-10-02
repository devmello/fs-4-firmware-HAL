#!/usr/bin/env bash
# Builds the upstream Mbed VCU firmware (fs-4-firmware/vcu) as a reference
# binary, out of tree, without writing to the reference checkout.
#
#   tools/mbed-ref/build.sh
#
# REF_DIR    fs-4-firmware checkout to build from (default ~/Projects/fs-4-ref)
# BUILD_DIR  output (default build/mbed-ref in this repo)
set -euo pipefail

repo=$(cd "$(dirname "$0")/../.." && pwd)
ref=${REF_DIR:-$HOME/Projects/fs-4-ref}
build=${BUILD_DIR:-$repo/build/mbed-ref}
venv=$repo/.venv

# Mbed would make its own venv in $ref/mbed-os/venv and write .vscode/ into
# $ref/vcu, so both are turned off below
if [ ! -x "$venv/bin/python" ]; then
    python3 -m venv "$venv"
fi
export PATH="$venv/bin:$PATH"

# Fail if the build touches the reference checkout, checked on exit so a
# failed build is caught too
ref_state() { git -C "$ref" status --porcelain --ignored; }
before=$(ref_state)
check_ref() {
    if [ "$(ref_state)" != "$before" ]; then
        echo "Build changed files in $ref:" >&2
        git -C "$ref" status --short --ignored >&2
        exit 1
    fi
}
trap check_ref EXIT
if ! "$venv/bin/python" -c "import mbed_tools.cli.cmsis_mcu_descr" 2>/dev/null; then
    # Install from a copy so pip doesn't build inside the reference checkout
    rm -rf "$build/mbed-ce-tools"
    mkdir -p "$build"
    cp -R "$ref/mbed-os/tools" "$build/mbed-ce-tools"
    "$venv/bin/pip" install --quiet "$build/mbed-ce-tools"
fi

cmake -S "$ref/vcu" -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Develop \
    -DMBED_CREATE_PYTHON_VENV=OFF \
    -DMBED_GENERATE_VSCODE_CONFIG=OFF \
    -DPython3_EXECUTABLE="$venv/bin/python"
cmake --build "$build"

bin=$build/vcu.bin
elf=$build/vcu.elf
arm-none-eabi-size "$elf"
shasum -a 256 "$bin"
