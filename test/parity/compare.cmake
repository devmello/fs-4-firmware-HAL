# cmake -DMBED=... -DHAL=... -DSTART_US=... -DOUT_DIR=... -P compare.cmake

set(mbed_out ${OUT_DIR}/mbed_${START_US}.txt)
set(hal_out ${OUT_DIR}/hal_${START_US}.txt)

execute_process(COMMAND ${MBED} ${START_US}
    OUTPUT_FILE ${mbed_out} ERROR_VARIABLE mbed_err RESULT_VARIABLE mbed_result)
execute_process(COMMAND ${HAL} ${START_US}
    OUTPUT_FILE ${hal_out} ERROR_VARIABLE hal_err RESULT_VARIABLE hal_result)
if(NOT mbed_result EQUAL 0 OR NOT hal_result EQUAL 0)
    message(FATAL_ERROR "a parity run failed (mbed ${mbed_result}, hal ${hal_result})\n"
        "mbed:\n${mbed_err}\nhal:\n${hal_err}")
endif()
message(STATUS "trace: ${mbed_err}")

execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${mbed_out} ${hal_out} RESULT_VARIABLE diff)
if(NOT diff EQUAL 0)
    set(where "")
    find_program(CMP cmp)
    if(CMP)
        execute_process(COMMAND ${CMP} ${mbed_out} ${hal_out} OUTPUT_VARIABLE where ERROR_QUIET)
    endif()
    message(FATAL_ERROR "Mbed and HAL outputs differ, see ${mbed_out} and ${hal_out}\n${where}")
endif()

# About 45 MB each, and identical
file(REMOVE ${mbed_out} ${hal_out})
