# Flash backup and restore over OpenOCD. Run by the backup-vcu and restore-vcu
# targets with cmake -P.
#
#   -DMODE=backup|restore -DOPENOCD=... -DCFG=... -DBOARD=vcu|nucleo
#   backup:  -DBACKUP_CFG=... -DOUT_DIR=...
#   restore: -DRESTORE_BIN=...
#
# Set DRY_RUN=1 in the environment to print the OpenOCD command without running it.

set(FLASH_BASE 0x08000000)
set(FLASH_SIZE 524288) # 0x80000

# Prints the command and runs it, unless DRY_RUN is set. Sets rc_var to its exit code.
function(run_openocd rc_var)
    set(shown "")
    foreach(arg IN LISTS ARGN)
        if(arg MATCHES " ")
            set(arg "\"${arg}\"")
        endif()
        string(APPEND shown " ${arg}")
    endforeach()
    string(STRIP "${shown}" shown)
    message(STATUS "${shown}")
    if("$ENV{DRY_RUN}")
        message(STATUS "DRY_RUN set, not running OpenOCD")
        set(${rc_var} 0 PARENT_SCOPE)
        return()
    endif()
    if(NOT OPENOCD)
        message(FATAL_ERROR "openocd not found (configure again after installing it)")
    endif()
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc)
    set(${rc_var} "${rc}" PARENT_SCOPE)
endfunction()

if(MODE STREQUAL "backup")
    string(TIMESTAMP stamp "%Y-%m-%d-%H%M%S")
    file(MAKE_DIRECTORY "${OUT_DIR}")
    set(out "${OUT_DIR}/${BOARD}-${stamp}.bin")
    # Dump to .part and rename only after the checks below, so a failed or bad
    # read never sits in backups/ looking like a backup
    set(part "${out}.part")

    # Halt without reset, dump, then let the firmware carry on. OpenOCD skips
    # the rest of the -c commands after one fails, so the dump is caught to
    # make sure resume still runs.
    run_openocd(rc "${OPENOCD}" -f "${BACKUP_CFG}"
        -c "init"
        -c "halt"
        -c "set r [catch {dump_image {${part}} ${FLASH_BASE} ${FLASH_SIZE}}]"
        -c "resume"
        -c "if {[set r]} {shutdown error}"
        -c "exit")
    if("$ENV{DRY_RUN}")
        return()
    endif()
    if(NOT rc EQUAL 0)
        file(REMOVE "${part}")
        message(FATAL_ERROR "OpenOCD failed (${rc}), no backup written")
    endif()

    file(SIZE "${part}" size)
    if(NOT size EQUAL FLASH_SIZE)
        file(REMOVE "${part}")
        message(FATAL_ERROR "Dump is ${size} bytes, expected ${FLASH_SIZE}, no backup written")
    endif()
    # First word is the initial stack pointer, somewhere in SRAM (0x2000xxxx).
    # All zeros means the chip was read while held in reset.
    file(READ "${part}" sp LIMIT 4 HEX)
    if(NOT sp MATCHES "^......20$")
        file(RENAME "${part}" "${out}.bad")
        message(FATAL_ERROR "Initial SP is ${sp} (bytes, little endian), not in SRAM. "
                            "Blank flash, or a bad read. Kept as ${out}.bad, not a backup.")
    endif()
    file(RENAME "${part}" "${out}")
    file(SHA256 "${out}" sha)
    message("${out}")
    message("sha256 ${sha}")

elseif(MODE STREQUAL "restore")
    if(NOT RESTORE_BIN)
        message(FATAL_ERROR "Set the file to flash: cmake -DRESTORE_BIN=path/to/file.bin --preset ...")
    endif()
    if(IS_DIRECTORY "${RESTORE_BIN}")
        message(FATAL_ERROR "RESTORE_BIN ${RESTORE_BIN} is a directory, not a .bin")
    endif()
    if(NOT EXISTS "${RESTORE_BIN}")
        message(FATAL_ERROR "RESTORE_BIN ${RESTORE_BIN} doesn't exist")
    endif()
    file(SIZE "${RESTORE_BIN}" size)
    if(size EQUAL 0 OR size GREATER FLASH_SIZE)
        message(FATAL_ERROR "${RESTORE_BIN} is ${size} bytes, must be 1 to ${FLASH_SIZE}")
    endif()
    file(SHA256 "${RESTORE_BIN}" sha)
    message(STATUS "Flashing ${RESTORE_BIN} (${size} bytes, sha256 ${sha}) to ${BOARD}")

    run_openocd(rc "${OPENOCD}" -f "${CFG}"
        -c "program {${RESTORE_BIN}} ${FLASH_BASE} verify reset exit")
    if(NOT rc EQUAL 0)
        message(FATAL_ERROR "OpenOCD failed (${rc})")
    endif()

else()
    message(FATAL_ERROR "MODE must be backup or restore")
endif()
