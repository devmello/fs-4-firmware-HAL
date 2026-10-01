# cmake -DMBED=... -DHAL=... -DSTART_US=... -DOUT_DIR=... -P compare.cmake

set(mbed_out ${OUT_DIR}/mbed_${START_US}.txt)
set(hal_out ${OUT_DIR}/hal_${START_US}.txt)

execute_process(COMMAND ${MBED} ${START_US} OUTPUT_FILE ${mbed_out} RESULT_VARIABLE mbed_result)
execute_process(COMMAND ${HAL} ${START_US} OUTPUT_FILE ${hal_out} RESULT_VARIABLE hal_result)
if(NOT mbed_result EQUAL 0 OR NOT hal_result EQUAL 0)
    message(FATAL_ERROR "a parity run failed (mbed ${mbed_result}, hal ${hal_result})")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${mbed_out} ${hal_out} RESULT_VARIABLE diff)
if(NOT diff EQUAL 0)
    message(FATAL_ERROR "Mbed and HAL outputs differ, see ${mbed_out} and ${hal_out}")
endif()
