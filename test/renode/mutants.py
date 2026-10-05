#!/usr/bin/env python3
"""
Runs run_tests.py against deliberately broken builds of the firmware, to see
which bugs it catches. Each mutant changes one thing in a copy of the tree in
--work (the real tree isn't touched), gets built with the debug preset there,
and the suite runs against its ELF.

    python3 test/renode/mutants.py --work /tmp/vcu-mutants
    python3 test/renode/mutants.py --work /tmp/vcu-mutants job_order no_copy

Scenarios run one at a time, shortest first, and a mutant stops at the first
one that fails (--all runs every scenario on every mutant). All 60 take about
30 minutes on a 10-core Mac; a mutant that isn't caught runs all 15 scenarios,
about 11 minutes. Failed runs keep their files in --work/runs. A mutant whose
text isn't in the source any more is reported as stale.
"""

import argparse
import concurrent.futures
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

# name, file, what it breaks, [(old, new[, nth]), ...] applied in order. Each
# old has to be in the file exactly once when applied, unless nth picks one.
MUTANTS = [
    ('job_order', 'vcu/main.cpp', '50 ms job before the 80 ms one', [
        ('        if (period_elapsed(now, next_data, DATA_PERIOD_US)) {\n'
         '            send_sme_CAN_messages_data();\n'
         '        }\n'
         '        if (period_elapsed(now, next_etc, ETC_CAN_PERIOD_US)) {\n'
         '            send_etc_CAN_messages();\n'
         '        }\n',
         '        if (period_elapsed(now, next_etc, ETC_CAN_PERIOD_US)) {\n'
         '            send_etc_CAN_messages();\n'
         '        }\n'
         '        if (period_elapsed(now, next_data, DATA_PERIOD_US)) {\n'
         '            send_sme_CAN_messages_data();\n'
         '        }\n'),
    ]),
    ('no_copy', 'vcu/main.cpp', 'no CAN_D copy of 390/646', [
        ('    data_copy_pending = true;',
         '    data_copy_pending = false;'),
    ]),
    ('copy_4ms', 'vcu/main.cpp', 'copies 4 ms after instead of 5', [
        ("DATA_COPY_DELAY_US = 5'000;",
         "DATA_COPY_DELAY_US = 4'000;"),
    ]),
    ('powertrain_50ms', 'vcu/main.cpp', '390/646 every 50 ms', [
        ("POWERTRAIN_PERIOD_US = 40'000;",
         "POWERTRAIN_PERIOD_US = 50'000;"),
    ]),
    ('tim5_wrap_not_counted', 'vcu/board/timebase.c', 'TIM5 wraps not counted', [
        ('        tim5_wraps++;\n',
         ''),
    ]),
    ('torque_bytes_swapped', 'vcu/main.cpp', 'torque bytes swapped in the 40 ms 390', [
        ('    // garbage on the bus). Zero here.\n'
         '    uint8_t tpdo_throttle_demand[8] = {0};\n'
         '    tpdo_throttle_demand[0] = etc_state.motor_torque.read() & 0xFF;\n'
         '    tpdo_throttle_demand[1] = etc_state.motor_torque.read() >> 8;',
         '    // garbage on the bus). Zero here.\n'
         '    uint8_t tpdo_throttle_demand[8] = {0};\n'
         '    tpdo_throttle_demand[1] = etc_state.motor_torque.read() & 0xFF;\n'
         '    tpdo_throttle_demand[0] = etc_state.motor_torque.read() >> 8;'),
    ]),
    ('alive_after_send', 'vcu/main.cpp', 'MBB_Alive incremented after the send', [
        ('void send_sme_CAN_messages_powertrain() {\n'
         '    etc.update_mbb_alive();\n',
         'void send_sme_CAN_messages_powertrain() {\n'),
        ('    can_send(CAN_P, 646, tpdo_max_currents, 8);\n',
         '    can_send(CAN_P, 646, tpdo_max_currents, 8);\n'
         '    etc.update_mbb_alive();\n'),
    ]),
    ('data80_order', 'vcu/main.cpp', '80 ms frames in the wrong order', [
        ('    can_send(CAN_D, 390, tpdo_throttle_demand, 8);\n'
         '    can_send(CAN_D, 660, tpdo_traction_data, 8);\n'
         '    can_send(CAN_D, 646, tpdo_max_currents, 8);',
         '    can_send(CAN_D, 390, tpdo_throttle_demand, 8);\n'
         '    can_send(CAN_D, 646, tpdo_max_currents, 8);\n'
         '    can_send(CAN_D, 660, tpdo_traction_data, 8);'),
    ]),
    ('stack_garbage', 'vcu/main.cpp', '646 bytes 4-7 uninitialized, as in the Mbed build', [
        ('    uint8_t tpdo_max_currents[8] = {0};',
         '    uint8_t tpdo_max_currents[8];', 1),
    ]),
    ('imu_accel_x10', 'vcu/main.cpp', 'accel X scaled x10', [
        ('etc_state.vectornav.accel[0] * 100',
         'etc_state.vectornav.accel[0] * 10'),
    ]),
    ('imu_lat_1e6', 'vcu/main.cpp', 'latitude scaled 1e6', [
        ('pos.lat * 1e7',
         'pos.lat * 1e6'),
    ]),
    ('imu_gyro_rad', 'vcu/main.cpp', 'gyro X sent in rad/s', [
        ('ang_rate[0] * RAD_TO_DEG * 10',
         'ang_rate[0] * 10'),
    ]),
    ('imu_frame_order', 'vcu/main.cpp', 'IMU frames in another order', [
        ('    can_send(CAN_D, 976, buf_ypr, 6);\n'
         '    can_send(CAN_D, 721, buf_latlon, 8);',
         '    can_send(CAN_D, 721, buf_latlon, 8);\n'
         '    can_send(CAN_D, 976, buf_ypr, 6);'),
    ]),
    ('debug_labels', 'vcu/main.cpp', 'debug line labels swapped', [
        ('| RTD %d EN %d |',
         '| EN %d RTD %d |'),
    ]),
    ('steering_angle', 'vcu/main.cpp', 'steering calibration', [
        ('STEERING_MAX_ANGLE = 75.82;',
         'STEERING_MAX_ANGLE = 75.0;'),
    ]),
    ('status_bit', 'vcu/main.cpp', '403 solenoid bit moved', [
        ('(etc_state.solenoid_open << 4)',
         '(etc_state.solenoid_open << 5)'),
    ]),
    ('rx_precharge_bit', 'vcu/main.cpp', '913 precharge read from the wrong bit', [
        ('rx.data[0] & 0b01000000;',
         'rx.data[0] & 0b00100000;'),
    ]),
    ('no_forward_1154', 'vcu/main.cpp', '1154 not forwarded', [
        ('        etc.update_regen_state(ground_speed);\n'
         '        can_send_frame(CAN_D, &rx);',
         '        etc.update_regen_state(ground_speed);'),
    ]),
    ('tray_45', 'vcu/main.cpp', 'tray temperature limit 45 C', [
        ('if (tray_temp > 40.0) {',
         'if (tray_temp > 45.0) {'),
    ]),
    ('wheels_swapped', 'vcu/main.cpp', 'front left and rear left wheel frames swapped', [
        ('wheel_rpm_fl = (rx.data[0]',
         'wheel_rpm_XX = (rx.data[0]'),
        ('wheel_rpm_bl = (rx.data[0]',
         'wheel_rpm_fl = (rx.data[0]'),
        ('wheel_rpm_XX = (rx.data[0]',
         'wheel_rpm_bl = (rx.data[0]'),
    ]),
    ('no_watchdog_refresh', 'vcu/main.cpp', 'watchdog never refreshed', [
        ('        watchdog_refresh();\n',
         ''),
    ]),
    ('can2_timing', 'vcu/board/can.c', 'CAN2 at 500 kbit/s', [
        ('bus_init(&buses[CAN_D], CAN2, 3,',
         'bus_init(&buses[CAN_D], CAN2, 6,'),
    ]),
    ('can2_filter_bank', 'vcu/board/can.c', "CAN2 filter in bank 13 (CAN1's)", [
        ('CAN_BS2_4TQ, 14, CAN2_TX_IRQn',
         'CAN_BS2_4TQ, 13, CAN2_TX_IRQn'),
    ]),
    ('can_no_pullup', 'vcu/board/can.c', 'no pull-up on the CAN pins', [
        ('gpio.Pull = GPIO_PULLUP; // like Mbed',
         'gpio.Pull = GPIO_NOPULL; // like Mbed'),
    ]),
    ('can_no_retransmit', 'vcu/board/can.c', 'automatic retransmission off', [
        ('hcan->Init.AutoRetransmission = ENABLE;',
         'hcan->Init.AutoRetransmission = DISABLE;'),
    ]),
    ('tx_no_pump', 'vcu/board/can.c', 'queued frames only sent from the TX interrupt', [
        ('    // queue along first instead of dropping a frame that would fit\n'
         '    tx_pump(bus);\n',
         '    // queue along first instead of dropping a frame that would fit\n'),
        ('        bus->tx_dropped++;\n'
         '    }\n'
         '    tx_pump(bus);',
         '        bus->tx_dropped++;\n'
         '    }'),
    ]),
    ('tx_pump_after_check', 'vcu/board/can.c', 'space checked before the queue is moved along', [
        ('    // queue along first instead of dropping a frame that would fit\n'
         '    tx_pump(bus);\n',
         '    // queue along first instead of dropping a frame that would fit\n'),
    ]),
    ('outputs_late', 'vcu/board/board.c', 'outputs set up after the clocks', [
        ('    gpio_init();\n'
         '    watchdog_init();\n'
         '    clock_init();',
         '    watchdog_init();\n'
         '    clock_init();\n'
         '    gpio_init();'),
    ]),
    ('reset_cause_order', 'vcu/board/board.c', 'reset pin flag checked first', [
        ('    if ((reset_flags & RCC_CSR_IWDGRSTF) != 0U) {\n'
         '        return "watchdog";\n'
         '    }',
         '    if ((reset_flags & RCC_CSR_PINRSTF) != 0U) {\n'
         '        return "reset pin";\n'
         '    }\n'
         '    if ((reset_flags & RCC_CSR_IWDGRSTF) != 0U) {\n'
         '        return "watchdog";\n'
         '    }'),
    ]),
    ('reset_flags_kept', 'vcu/board/board.c', 'reset flags never cleared', [
        ('    RCC->CSR |= RCC_CSR_RMVF;\n',
         ''),
    ]),
    ('error_handler_silent', 'vcu/board/board.c', 'Error_Handler prints nothing', [
        ('    console_write_blocking(message);',
         '    (void)message;'),
    ]),
    ('rtd_falling_edge', 'vcu/board/gpio.c', 'RTD button on the falling edge', [
        ('#define RTD_BUTTON_EDGE   GPIO_MODE_IT_RISING',
         '#define RTD_BUTTON_EDGE   GPIO_MODE_IT_FALLING'),
    ]),
    ('exti_off', 'vcu/board/gpio.c', 'RTD button interrupt not enabled', [
        ('    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);\n',
         ''),
    ]),
    ('bpps_channel', 'vcu/board/adc.h', 'BPPS read from channel 14', [
        ('#define ADC_CH_BPPS      13U',
         '#define ADC_CH_BPPS      14U'),
    ]),
    ('adc_sampling', 'vcu/board/adc.c', 'ADC sampling 15 cycles', [
        ('ADC_SAMPLETIME_56CYCLES',
         'ADC_SAMPLETIME_15CYCLES'),
    ]),
    ('watchdog_500ms', 'vcu/board/watchdog.c', 'watchdog timeout 500 ms', [
        ('hiwdg.Init.Reload = 125U - 1U;',
         'hiwdg.Init.Reload = 250U - 1U;'),
    ]),
    ('dma_normal', 'vcu/board/imu_uart.c', 'UART5 RX DMA not circular', [
        ('hdma_rx.Init.Mode = DMA_CIRCULAR;',
         'hdma_rx.Init.Mode = DMA_NORMAL;'),
    ]),
    ('dma_channel', 'vcu/board/imu_uart.c', 'UART5 RX DMA on channel 5', [
        ('hdma_rx.Init.Channel = DMA_CHANNEL_4;',
         'hdma_rx.Init.Channel = DMA_CHANNEL_5;'),
    ]),
    ('dma_no_half', 'vcu/board/imu_uart.c', 'no half transfer interrupt', [
        ('    hdma_rx.XferHalfCpltCallback = rx_half_done;',
         '    hdma_rx.XferHalfCpltCallback = NULL;'),
    ]),
    ('uart5_baud', 'vcu/board/imu_uart.c', 'UART5 at 57600', [
        ('huart5.Init.BaudRate = 115200;',
         'huart5.Init.BaudRate = 57600;'),
    ]),
    ('no_deadzone_clamp', 'vcu/etc/etc_controller.cpp', "no clamp before the LUT (upstream's out of bounds read)", [
        ('    if (pedal_travel < 0.0f) {\n'
         '        pedal_travel = 0.0f;\n'
         '    }\n',
         ''),
    ]),
    ('implaus_50ms', 'vcu/etc/etc_controller.cpp', 'implausibility after 50 ms', [
        ('if (time_ms_elapsed > 100) {',
         'if (time_ms_elapsed > 50) {'),
    ]),
    ('implaus_ge_100', 'vcu/etc/etc_controller.cpp', 'implausibility at 100 ms instead of 101', [
        ('if (time_ms_elapsed > 100) {',
         'if (time_ms_elapsed >= 100) {'),
    ]),
    ('latch_clear_10', 'vcu/etc/etc_controller.cpp', 'brake + accel clears under 10%', [
        ('state.APPS_position_avg < 0.05f) {',
         'state.APPS_position_avg < 0.10f) {'),
    ]),
    ('brakelight_20', 'vcu/etc/etc_controller.cpp', 'brake light at 20 psi', [
        ('state.brakelight_enabled = (state.front_BSE_pressure > 30);',
         'state.brakelight_enabled = (state.front_BSE_pressure > 20);'),
    ]),
    ('rtd_brake_5', 'vcu/etc/etc_controller.h', 'RTD needs 5% brake', [
        ('BPPS_BRAKE_ENGAGE_PERCENT = 0.09f;',
         'BPPS_BRAKE_ENGAGE_PERCENT = 0.05f;'),
    ]),
    ('off_stops_buzzer', 'vcu/etc/etc_controller.cpp', 'turning RTD off also silences the buzzer (not upstream)', [
        ('    state.ready_to_drive = false;\n'
         '    rtd_light.write(0);\n'
         '}',
         '    state.ready_to_drive = false;\n'
         '    rtd_light.write(0);\n'
         '    rtd_buzzer.write(0);\n'
         '}'),
    ]),
    ('buzzer_never_off', 'vcu/etc/etc_controller.cpp', 'buzzer not turned off', [
        ('[this] { rtd_buzzer.write(0); }',
         '[this] { }'),
    ]),
    ('buzzer_1s', 'vcu/etc/etc_controller.h', 'buzzer for 1 s', [
        ('RTD_BUZZER_DURATION{2}',
         'RTD_BUZZER_DURATION{1}'),
    ]),
    ('deviation_15', 'vcu/etc/etc_controller.h', 'APPS deviation limit 15%', [
        ('MAX_APPS_POSITION_DEVIATION = 0.10f;',
         'MAX_APPS_POSITION_DEVIATION = 0.15f;'),
    ]),
    ('bse_buffer', 'vcu/etc/etc_controller.h', 'front BSE range buffer 0.03 V', [
        ('FRONT_BSE_BUFFER_VOLTAGE = 0.02f;',
         'FRONT_BSE_BUFFER_VOLTAGE = 0.03f;'),
    ]),
    ('lut_typo', 'vcu/etc/etc_controller.cpp', 'one LUT point off', [
        ('        0.350000000f,',
         '        0.360000000f,'),
    ]),
    ('max_torque', 'vcu/etc/etc_controller.h', 'torque scale 0.60', [
        ('MAX_TORQUE = 32767*0.65;',
         'MAX_TORQUE = 32767*0.60;'),
    ]),
    ('torque_filter_20hz', 'vcu/etc/etc_controller.h', 'torque filter at 20 Hz', [
        ('motor_torque{40}',
         'motor_torque{20}'),
    ]),
    ('apps_filter_30hz', 'vcu/etc/etc_controller.cpp', 'APPS1 filter at 30 Hz', [
        ('APPS1_input(unfiltered_APPS1_input, 60)',
         'APPS1_input(unfiltered_APPS1_input, 30)'),
    ]),
    ('slip_front_ref', 'vcu/etc/traction_control.cpp', 'slip relative to the front wheels', [
        ('slip = (v_rear - v_front) / v_rear;',
         'slip = (v_rear - v_front) / v_front;'),
    ]),
    ('activation_50', 'vcu/etc/traction_control.h', 'traction control from 50 rpm', [
        ('ACTIVATION_RPM = 100.0f;',
         'ACTIVATION_RPM = 50.0f;'),
    ]),
    ('vn_timeout_1s', 'vcu/imu/vn200.h', 'VN-200 reconfigured after 1 s without data', [
        ("DATA_TIMEOUT_US = 500'000;",
         "DATA_TIMEOUT_US = 1'000'000;"),
    ]),
    ('vn_response_200', 'vcu/imu/vn200.h', 'VN-200 command timeout 200 ms', [
        ("RESPONSE_TIMEOUT_US = 100'000;",
         "RESPONSE_TIMEOUT_US = 200'000;"),
    ]),
    ('vn_checksum', 'vcu/imu/vn200.cpp', 'command checksum summed instead of XORed', [
        ('        sum ^= static_cast<uint8_t>(s[i]);',
         '        sum += static_cast<uint8_t>(s[i]);'),
    ]),
    ('vn_no_crc', 'vcu/imu/vn200.cpp', 'binary CRC not checked', [
        ('if (crc16(pkt + i + 1, PACKET_SIZE - 1) != 0) {',
         'if (false) {'),
    ]),
    ('vn_no_data_refresh', 'vcu/imu/vn200.cpp', "good packets don't hold off the data timeout", [
        ('    last_packet_time = now_us;\n'
         '    last_data_us = now_us;\n',
         '    last_packet_time = now_us;\n'),
    ]),
]


def apply(text, changes, name):
    for change in changes:
        old, new = change[0], change[1]
        nth = change[2] if len(change) > 2 else None
        count = text.count(old)
        if nth is None and count != 1:
            return None, f"{name}: text found {count} times: {old[:60]!r}"
        if nth is not None and count < nth:
            return None, f"{name}: text found {count} times, wanted occurrence {nth}"
        at = -1
        for _ in range(nth or 1):
            at = text.index(old, at + 1)
        text = text[:at] + new + text[at + len(old):]
    return text, None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--work", required=True, help="scratch directory (gets a copy of the tree)")
    parser.add_argument("--parallel", type=int, default=max(1, min(8, (os.cpu_count() or 4) // 2)),
                        help="mutants run at once")
    parser.add_argument("--all", action="store_true", help="don't stop at the first failing scenario")
    parser.add_argument("only", nargs="*")
    args = parser.parse_args()

    todo = [m for m in MUTANTS if not args.only or m[0] in args.only]
    if len(todo) != len(set(args.only or [m[0] for m in MUTANTS])):
        sys.exit(f"unknown mutant in {args.only}")
    tree = os.path.join(args.work, "tree")
    elves = os.path.join(args.work, "elf")
    logs = os.path.join(args.work, "logs")
    runs = os.path.join(args.work, "runs")  # the suite keeps a failed run's files, here instead of /tmp
    # Refreshed every time; the copy's build directory is kept
    shutil.copytree(ROOT, tree, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("build", ".git", ".venv", "backups", "__pycache__"))
    os.makedirs(elves, exist_ok=True)
    os.makedirs(logs, exist_ok=True)
    os.makedirs(runs, exist_ok=True)

    def build():
        for cmd in (["cmake", "--preset", "debug"], ["cmake", "--build", "--preset", "debug"]):
            p = subprocess.run(cmd, cwd=tree, capture_output=True, text=True)
            if p.returncode:
                return p.stdout + p.stderr
        return None

    error = build()
    if error:
        sys.exit(f"the unmodified copy doesn't build:\n{error}")

    results = {}
    built = []
    for name, path, what, changes in todo:
        source = os.path.join(tree, path)
        original = open(source).read()
        mutated, problem = apply(original, changes, name)
        if problem:
            results[name] = ("stale", problem, [])
            continue
        try:
            open(source, "w").write(mutated)
            error = build()
        finally:
            open(source, "w").write(original)
        if error:
            results[name] = ("doesn't build", error.strip().splitlines()[-1], [])
            continue
        shutil.copy(os.path.join(tree, "build", "debug", "vcu", "vcu.elf"), os.path.join(elves, name + ".elf"))
        built.append(name)
    build()  # leave the copy built unmodified

    # The copy's suite, so editing the real one doesn't affect a run
    suite = os.path.join(tree, "test", "renode", "run_tests.py")
    listing = subprocess.run([sys.executable, suite, "--list"], capture_output=True, text=True, check=True).stdout
    lengths = {line.split()[0]: float(line.split()[1]) for line in listing.splitlines() if line.strip()}
    order = sorted(lengths, key=lengths.get)

    def run(name):
        failed, first, log = [], "", []
        for scenario in order:
            p = subprocess.run([sys.executable, suite, "--elf", os.path.join(elves, name + ".elf"), "--jobs", "1",
                                scenario], capture_output=True, text=True, env=dict(os.environ, TMPDIR=runs))
            out = p.stdout + p.stderr
            log.append(out)
            if p.returncode == 0:
                continue
            message = re.search(r"^FAIL \S+.*\n\s+(.+)", out, re.M)
            if not message:  # the suite itself failed, e.g. a symbol it needs is gone
                open(os.path.join(logs, name + ".txt"), "w").write("\n".join(log))
                return name, "suite error", (out.strip().splitlines() or ["?"])[-1], failed
            failed.append(scenario)
            first = first or message.group(1)
            if not args.all:
                break
        open(os.path.join(logs, name + ".txt"), "w").write("\n".join(log))
        return name, "caught" if failed else "MISSED", first, failed

    with concurrent.futures.ThreadPoolExecutor(args.parallel) as pool:
        for name, status, first, failed in pool.map(run, built):
            results[name] = (status, first, failed)
            print(f"{status:8} {name:22} {', '.join(failed)}", flush=True)

    print()
    caught = 0
    for name, _, what, _ in todo:
        status, detail, failed = results[name]
        caught += status == "caught"
        print(f"{status:13} {name:22} {what}")
        if failed:
            print(f"{'':36}by {', '.join(failed)}")
        if detail:
            print(f"{'':36}{detail[:150]}")
    print(f"\n{caught}/{len(todo)} caught. Logs in {logs}")


if __name__ == "__main__":
    main()
