# Support for mechaduino style "servo steppers"
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import bus
import chelper

UPDATE_TIME = 1. / 6000
CALIBRATION_COUNT = 200

class error(Exception):
    pass

######################################################################
# Low-level mcu wrappers
######################################################################

# Stepper motor h-bridge driver
class MCU_a4954:
    def __init__(self, config):
        self.printer = config.get_printer()
        # pin config
        ppins = self.printer.lookup_object('pins')
        pins = [ppins.lookup_pin(config.get(name + '_pin'))
                for name in ['in1', 'in2', 'in3', 'in4', 'vref12', 'vref34']]
        mcu = None
        for pin_params in pins:
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("mechaduino: all pins must be on same mcu")
            mcu = pin_params['chip']
        self.pins = [pin_params['pin'] for pin_params in pins]
        self.mcu = mcu
        self.oid = self.mcu.create_oid()
        self.mcu.add_config_cmd(
            "config_a4954 oid=%d in1_pin=%s in2_pin=%s in3_pin=%s in4_pin=%s"
            " vref12_pin=%s vref34_pin=%s" % (
                self.oid, self.pins[0], self.pins[1],
                self.pins[2], self.pins[3], self.pins[4], self.pins[5]))
        # Current control
        sense_resistor = config.getfloat('sense_resistor', above=0.)
        vref = config.getfloat('voltage_reference', 3.3, above=0.)
        self.current_factor = 10. * sense_resistor / vref
        self.max_current = config.getfloat('current', above=0., maxval=2.)
        self.hold_current = config.getfloat(
            'hold_current', self.max_current, above=0.,
            maxval=self.max_current)
        self.out_max = 0.
        self.mcu.register_config_callback(self._build_config)
    def _build_config(self):
        try:
            # The DAC is preferred if available
            self.out_max = self.mcu.get_constant_float("DAC_MAX")
        except Exception:
            self.out_max = self.mcu.get_constant_float("PWM_MAX")
        logging.info("Mechaduino: Maximum Output Scale = %.2f" % self.out_max)
    def get_mcu(self):
        return self.mcu
    def get_oid(self):
        return self.oid
    def get_current_scale(self, current=None):
        if current is None:
            current = self.max_current
        current = max(0., min(self.max_current, current))
        return min(self.out_max,
                   int(current * self.current_factor * self.out_max))

# Virtual stepper position tracking class
# XXX - this is just a dup of mcu.MCU_stepper with different mcu commands
class MCU_virtual_stepper:
    def __init__(self, mcu, name, invert_dir, step_dist,
                 units_in_radians=False):
        self._mcu = mcu
        self._name = name
        self._step_dist = step_dist
        self._units_in_radians = units_in_radians
        self._invert_dir = invert_dir
        self._oid = oid = self._mcu.create_oid()
        self._mcu.add_config_cmd("config_virtual_stepper oid=%d" % (self._oid,))
        self._mcu.register_config_callback(self._build_config)
        self._mcu_position_offset = self._tag_position = 0.
        self._min_stop_interval = 0.
        self._reset_cmd_id = self._get_position_cmd = None
        self._active_callbacks = []
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(self._ffi_lib.stepcompress_alloc(oid),
                                      self._ffi_lib.stepcompress_free)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._itersolve_generate_steps = self._ffi_lib.itersolve_generate_steps
        self._itersolve_check_active = self._ffi_lib.itersolve_check_active
        self._trapq = ffi_main.NULL
    def get_mcu(self):
        return self._mcu
    def get_name(self, short=False):
        if short and self._name.startswith('stepper_'):
            return self._name[8:]
        return self._name
    def set_units(self, units_in_radians):
        self._units_in_radians = units_in_radians
    def units_in_radians(self):
        # Returns true if distances are in radians instead of millimeters
        return self._units_in_radians
    def _dist_to_time(self, dist, start_velocity, accel):
        # Calculate the time it takes to travel a distance with constant accel
        time_offset = start_velocity / accel
        return math.sqrt(2. * dist / accel + time_offset**2) - time_offset
    def set_max_jerk(self, max_halt_velocity, max_accel):
        pass
    def setup_itersolve(self, alloc_func, *params):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self.set_stepper_kinematics(sk)
    def _build_config(self):
        max_error = self._mcu.get_max_stepper_error()
        self._mcu.add_config_cmd("virtual_reset_step_clock oid=%d clock=0" % (
            self._oid,), is_init=True)
        step_cmd_id = self._mcu.lookup_command_id(
            "virtual_queue_step oid=%c interval=%u count=%hu add=%hi")
        dir_cmd_id = self._mcu.lookup_command_id(
            "virtual_set_next_step_dir oid=%c dir=%c")
        self._reset_cmd_id = self._mcu.lookup_command_id(
            "virtual_reset_step_clock oid=%c clock=%u")
        self._get_position_cmd = self._mcu.lookup_query_command(
            "virtual_stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i", oid=self._oid)
        self._ffi_lib.stepcompress_fill(
            self._stepqueue, self._mcu.seconds_to_clock(max_error),
            self._invert_dir, step_cmd_id, dir_cmd_id)
    def get_oid(self):
        return self._oid
    def get_step_dist(self):
        return self._step_dist
    def is_dir_inverted(self):
        return self._invert_dir
    def calc_position_from_coord(self, coord):
        return self._ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
    def set_position(self, coord):
        opos = self.get_commanded_position()
        sk = self._stepper_kinematics
        self._ffi_lib.itersolve_set_position(sk, coord[0], coord[1], coord[2])
        self._mcu_position_offset += opos - self.get_commanded_position()
    def get_commanded_position(self):
        return self._ffi_lib.itersolve_get_commanded_pos(
            self._stepper_kinematics)
    def set_commanded_position(self, pos):
        self._mcu_position_offset += self.get_commanded_position() - pos
        self._ffi_lib.itersolve_set_commanded_pos(self._stepper_kinematics, pos)
    def get_mcu_position(self):
        mcu_pos_dist = self.get_commanded_position() + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self._step_dist
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    def get_tag_position(self):
        return self._tag_position
    def set_tag_position(self, position):
        self._tag_position = position
    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        self._stepper_kinematics = sk
        if sk is not None:
            self._ffi_lib.itersolve_set_stepcompress(
                sk, self._stepqueue, self._step_dist)
        return old_sk
    def note_homing_end(self, did_trigger=False):
        ret = self._ffi_lib.stepcompress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in stepcompress")
        data = (self._reset_cmd_id, self._oid, 0)
        ret = self._ffi_lib.stepcompress_queue_msg(
            self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        if not did_trigger or self._mcu.is_fileoutput():
            return
        params = self._get_position_cmd.send([self._oid])
        mcu_pos_dist = params['pos'] * self._step_dist
        if self._invert_dir:
            mcu_pos_dist = -mcu_pos_dist
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()
    def set_trapq(self, tq):
        if tq is None:
            ffi_main, self._ffi_lib = chelper.get_ffi()
            tq = ffi_main.NULL
        self._ffi_lib.itersolve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq
    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)
    def generate_steps(self, flush_time):
        # Check for activity if necessary
        if self._active_callbacks:
            ret = self._itersolve_check_active(self._stepper_kinematics,
                                               flush_time)
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        # Generate steps
        ret = self._itersolve_generate_steps(self._stepper_kinematics,
                                             flush_time)
        if ret:
            raise error("Internal error in stepcompress")
    def is_active_axis(self, axis):
        return self._ffi_lib.itersolve_is_active_axis(
            self._stepper_kinematics, axis)
    def get_realtime_position(self):
        params = self._get_position_cmd.send([self._oid])
        return params['pos']

# Servo stepper feedback control code
class MCU_servo_stepper:
    def __init__(self, config, stepper_driver, virtual_stepper):
        self.mcu = stepper_driver.get_mcu()
        self.stepper_driver = stepper_driver
        self.oid = self.mcu.create_oid()
        self.mcu_freq = None
        modes = {'open_loop': 'open_loop', 'hpid': 'hpid'}
        self.servo_mode = config.getchoice('mode', modes, 'open_loop')
        steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
                 '8': 5, '4': 6, '2': 7, '1': 8}
        microsteps = config.getchoice('microsteps', steps)
        self.step_multiplier = 1 << microsteps
        self.Kp = config.getfloat('pid_Kp', 1.0)
        self.Ki = config.getfloat('pid_Ki', .0)
        self.Kd = config.getfloat('pid_Kd', .0)
        self.full_steps_per_rotation = config.getint(
            'full_steps_per_rotation', 200, minval=4)
        self.mcu.add_config_cmd(
            "config_servo_stepper oid=%d driver_oid=%d stepper_oid=%d"
            " full_steps_per_rotation=%d step_multiplier=%d" % (
                self.oid, stepper_driver.get_oid(), virtual_stepper.get_oid(),
                self.full_steps_per_rotation, self.step_multiplier))
        # Commands
        self.set_mode_cmd = self.get_stats_cmd = None
        self.mcu.register_config_callback(self._build_config)
    def get_oid(self):
        return self.oid
    def get_full_steps_per_rotation(self):
        return self.full_steps_per_rotation
    def _build_config(self):
        self.mcu_freq = self.mcu.get_constant_float('CLOCK_FREQ')
        cmd_queue = self.mcu.alloc_command_queue()
        self.set_mode_cmd = self.mcu.lookup_command(
            "servo_stepper_set_mode oid=%c mode=%c run_current_scale=%u"
            " flex=%u kp=%hi ki=%hi kd=%hi", cq=cmd_queue)
        self.get_stats_cmd = self.mcu.lookup_query_command(
            "servo_stepper_get_stats oid=%c",
            "servo_stepper_stats oid=%c error=%i max_time=%u",
            oid=self.oid, cq=cmd_queue)
    def set_disabled(self, print_time):
        clock = self.mcu.print_time_to_clock(print_time)
        self.set_mode_cmd.send(
            [self.oid, 0, 0, 0, 0, 0, 0], minclock=clock, reqclock=clock)
    def set_open_loop_mode(self, print_time):
        clock = self.mcu.print_time_to_clock(print_time)
        rc_scale = self.stepper_driver.get_current_scale()
        hc_scale = self.stepper_driver.get_current_scale(
            self.stepper_driver.hold_current)
        self.set_mode_cmd.send(
            [self.oid, 1, rc_scale, hc_scale, 0, 0, 0],
            minclock=clock, reqclock=clock)
    def set_torque_mode(self, print_time, excite_angle, current):
        clock = self.mcu.print_time_to_clock(print_time)
        current_scale = self.stepper_driver.get_current_scale(current)
        self.set_mode_cmd.send(
            [self.oid, 2, current_scale, excite_angle, 0, 0, 0],
            minclock=clock, reqclock=clock)
    def set_hpid_mode(self, print_time):
        clock = self.mcu.print_time_to_clock(print_time)
        rc_scale = self.stepper_driver.get_current_scale()
        hc_scale = self.stepper_driver.get_current_scale(
            self.stepper_driver.hold_current)
        kp = int(self.Kp * 1024)
        ki = int(self.Ki * 1024)
        kd = int(self.Kd * 1024)
        self.set_mode_cmd.send(
            [self.oid, 3, rc_scale, hc_scale, kp, ki, kd],
            minclock=clock, reqclock=clock)
    def set_servo_mode(self, mode):
        old_mode = self.servo_mode
        self.servo_mode = mode
        return old_mode
    def get_servo_mode(self):
        return self.servo_mode
    def get_servo_stats(self):
        params = self.get_stats_cmd.send([self.oid])
        return (params['error'], params.get('max_time'))


AS5047_REGS = {
    'NOP': 0xC000, 'EERFL': 0x4001, 'PROG': 0xC003,
    'DIAAGC': 0xFFFC, 'MAG': 0x7FFD, 'ANGLEUNC': 0x7FFE,
    'ANGLECOM': 0xFFFF
}

class MCU_as5047d:
    def __init__(self, config, spi):
        self.name = config.get_name().split()[-1]
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.spi = spi
        reactor = self.printer.get_reactor()
        self.mutex = reactor.mutex()

        # Register Gcode commands
        self.gcode.register_mux_command(
            "QUERY_ENCODER_POSITION", "SERVO", self.name,
            self.cmd_QUERY_ENCODER_POSITION)
        self.gcode.register_mux_command(
            "DUMP_ENCODER_REGS", "SERVO", self.name,
            self.cmd_DUMP_ENCODER_REGS)
    def query_position(self):
        bits = self.send_command('ANGLEUNC')
        val = (bits[0] << 8) | bits[1]
        return val & 0x3FFF
    def send_command(self, reg_name):
        reg = AS5047_REGS.get(reg_name)
        if reg is None:
            return None
        with self.mutex:
            self.spi.spi_send([(reg >> 8 & 0xFF), (reg & 0xFF)])
            params = self.spi.spi_transfer([0xC0, 0x00])
        return bytearray(params['response'])
    def cmd_QUERY_ENCODER_POSITION(self, params):
        pos = self.query_position() << 2
        self.gcode.respond_info('Encoder Position: %d' % (pos))
    def cmd_DUMP_ENCODER_REGS(self, params):
        msg = "%s AS5047 REGS:" % self.name.upper()
        for reg_name in AS5047_REGS:
            if reg_name == 'NOP':
                continue
            msg += '\n'
            bits = self.send_command(reg_name)
            val = (bits[0] << 8) | bits[1]
            msg += "%-10s %s" % (reg_name + ":", format(val, '#018b')[2:])
        self.gcode.respond_info(msg)

# SPI controlled hall position sensor
class MCU_spi_position:
    def __init__(self, config, control):
        self.control = control
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gc_response_counter = None
        # Sensor type
        sensors = { "a1333": (1, 3, 10000000),
                    "as5047d": (2, 1, 10000000) }
        self.sensor_type = config.getchoice(
            'sensor_type', {s: s for s in sensors})
        chip_id, spi_mode, spi_speed = sensors[self.sensor_type]
        # SPI bus configuration
        self.spi = bus.MCU_SPI_from_config(
            config, spi_mode, pin_option="sensor_pin", default_speed=spi_speed)
        self.mcu = self.spi.get_mcu()
        # Sensor chip configuration
        self.oid = self.mcu.create_oid()
        self.mcu.add_config_cmd(
            "config_spi_position oid=%d spi_oid=%d chip_type=%d"
            " servo_stepper_oid=%d" % (
                self.oid, self.spi.get_oid(), chip_id, self.control.get_oid()))
        self.autostart = config.getboolean('autostart_spi', True)
        if self.sensor_type == 'as5047d' and not self.autostart:
            MCU_as5047d(config, self.spi)
        # Commands
        self.update_clock = 0
        self.query_pos_cmd = self.set_calibration_cmd = None
        self.mcu.register_config_callback(self._build_config)
        # Position storage
        self.positions = []
    def _build_config(self):
        if self.autostart:
            clock = self.mcu.get_query_slot(self.oid)
            self.update_clock = self.mcu.seconds_to_clock(UPDATE_TIME)
            self.mcu.add_config_cmd(
                "schedule_spi_position oid=%u clock=%u rest_ticks=%u" % (
                    self.oid, clock, self.update_clock), is_init=True)
        cmd_queue = self.spi.get_command_queue()
        self.query_pos_cmd = self.mcu.lookup_command(
            "query_last_spi_position oid=%c", cq=cmd_queue)
        self.set_calibration_cmd = self.mcu.lookup_command(
            "set_spi_position_calibration oid=%c index=%u value=%hu",
            cq=cmd_queue)
    def _handle_spi_position_result(self, params):
        next_clock = self.mcu.clock32_to_clock64(params['next_clock'])
        last_read_clock = next_clock - self.update_clock
        last_read_time = self.mcu.clock_to_print_time(last_read_clock)
        position = params['position']
        self.positions.append((last_read_time, position))
        if self.gc_response_counter is not None:
            if len(self.positions) % self.gc_response_counter == 0:
                reactor = self.printer.get_reactor()
                reactor.register_async_callback(
                    lambda e, s=self: s.gcode.respond_info("Calibrating..."))
    def init_position_query(self, response_counter=None):
        self.gc_response_counter = response_counter
        self.positions = []
        self.mcu.register_response(
            self._handle_spi_position_result, "spi_position_result", self.oid)
    def query_position(self, print_time):
        clock = self.mcu.print_time_to_clock(print_time)
        self.query_pos_cmd.send([self.oid], minclock=clock, reqclock=clock)
    def get_clear_positions(self):
        self.gc_response_counter = None
        self.mcu.register_response(
            None, "spi_position_result", self.oid)
        pos = self.positions
        self.positions = []
        return list(pos)
    def get_position(self):
        reactor = self.printer.get_reactor()
        completion = reactor.completion()

        def response_handler(params):
            reactor.async_complete(completion, params)
        self.mcu.register_response(
            response_handler, "spi_position_result", self.oid)
        self.query_pos_cmd.send([self.oid])
        waketime = reactor.monotonic() + 1.
        params = completion.wait(waketime=waketime)
        self.mcu.register_response(
            None, "spi_position_result", self.oid)
        if params is not None:
            return (params['position'], params.get('interp_ticks'))
        else:
            return None, None
    def apply_calibration(self, print_time, calibration, reversed_cal):
        calibration.append(int(reversed_cal))
        for i, cal in enumerate(calibration):
            if self.set_calibration_cmd is None:
                self.mcu.add_config_cmd(
                    "set_spi_position_calibration oid=%d index=%d value=%d" % (
                        self.oid, i, cal & 0xFFFF), is_init=True)
            else:
                clock = self.mcu.print_time_to_clock(print_time)
                self.set_calibration_cmd.send([self.oid, i, cal & 0xFFFF],
                                              minclock=clock, reqclock=clock)


######################################################################
# Calibration
######################################################################

class ServoCalibration:
    def __init__(self, config, mcu_vstepper, servo_stepper, spi_position):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reversed_cal = config.getboolean('reversed_cal', False)
        self.mcu_vstepper = mcu_vstepper
        self.servo_stepper = servo_stepper
        self.spi_position = spi_position
        self.load_calibration(config)
        # Register commands
        servo_name = config.get_name().split()[1]
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("SERVO_CALIBRATE", "SERVO", servo_name,
                                        self.cmd_SERVO_CALIBRATE,
                                        desc=self.cmd_SERVO_CALIBRATE_help)
    def get_base_calibration(self):
        bucket_size = (65536 * 1024) // CALIBRATION_COUNT
        return [(i * bucket_size) // 1024 for i in range(CALIBRATION_COUNT)]
    def scale_calibration(self, cal):
        new_cal = []
        if len(cal) < CALIBRATION_COUNT:
            if CALIBRATION_COUNT % len(cal):
                # TODO: WE will have to interpolate the points
                # As it is not an even divisor
                raise NotImplementedError(
                    "Extrapolation not yet implemented")
            multiplier = CALIBRATION_COUNT // len(cal)
            new_cal.append(cal[0])
            for i in range(len(cal) - 1):
                delta_cal = cal[i + 1] - cal[i]
                for j in range(1, multiplier, 1):
                    adj = float(j) / multiplier * delta_cal
                    val = int(cal[i] + adj + .5)
                    new_cal.append(val)
                new_cal.append(cal[i + 1])
        else:
            if len(cal) % CALIBRATION_COUNT:
                # TODO: WE will have to interpolate the points
                # As it is not an even divisor
                raise NotImplementedError(
                    "Extrapolation not yet implemented")
            divisor = len(cal) // CALIBRATION_COUNT
            new_cal = [c for i, c in enumerate(cal) if not i % divisor]
        return new_cal
    def load_calibration(self, config):
        cal = config.get('calibrate', None)
        if cal is None:
            self.reset_calibration(0.)
            return
        # Parse calibration data
        data = [d.strip() for d in cal.split(',')]
        angles = [float(d) for d in data if d]
        calibration = [int(a + .5) for a in angles]
        if len(calibration) != 200:
            calibration = self.scale_calibration(calibration)
        self.spi_position.apply_calibration(0., calibration, self.reversed_cal)
    def reset_calibration(self, print_time):
        bc = self.get_base_calibration()
        self.reversed_cal = False
        self.spi_position.apply_calibration(print_time, bc, False)
    cmd_SERVO_CALIBRATE_help = "Calibrate the servo stepper"
    def cmd_SERVO_CALIBRATE(self, params):
        self.spi_position.init_position_query(response_counter=100)
        full_steps = self.servo_stepper.get_full_steps_per_rotation()
        # Go into open loop mode
        old_mode = self.servo_stepper.set_servo_mode("open_loop")
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        self.reset_calibration(print_time)
        # Start with a dummy movement
        fmove = self.printer.lookup_object('force_move')
        step_dist = self.mcu_vstepper.get_step_dist()
        full_step_dist = step_dist * self.servo_stepper.step_multiplier
        move_time = 0.100
        move_speed = full_step_dist / move_time
        fmove.manual_move(
            self.mcu_vstepper, 16. * -full_step_dist, move_speed)
        # Move to each full step position and then query the sensor
        steps = []
        for i in range(full_steps - 1, -1, -1):
            fmove.manual_move(
                self.mcu_vstepper, -full_step_dist, move_speed)
            start_query_time = toolhead.get_last_move_time() + 0.050
            for j in range(10):
                self.spi_position.query_position(start_query_time + j*0.010)
            end_query_time = start_query_time + 10 * 0.010
            steps.append((i, start_query_time, end_query_time))
            toolhead.dwell(0.050 + end_query_time - start_query_time)
        stepper_enable = self.printer.lookup_object('stepper_enable')
        se = stepper_enable.lookup_enable(self.mcu_vstepper.get_name())
        se.motor_disable(toolhead.get_last_move_time())
        self.servo_stepper.set_servo_mode(old_mode)
        toolhead.wait_moves()
        # Correlate query responses
        cal = {}
        positions = self.spi_position.get_clear_positions()
        step = 0
        for query_time, pos in positions:
            while step < len(steps) and query_time > steps[step][2]:
                step += 1
            if step < len(steps) and query_time >= steps[step][1]:
                cal.setdefault(steps[step][0], []).append(pos & 0xffff)
        # Calculate each step position average and variance
        total_count = total_variance = 0
        angles = {}
        for step, data in cal.items():
            count = len(data)
            angle_avg = float(sum(data)) / count
            angles[step] = angle_avg
            total_count += count
            total_variance += sum([(d - angle_avg)**2 for d in data])
        # Validate data
        angles_to_step = {a: s for s, a in angles.items()}
        if len(angles_to_step) != full_steps:
            raise self.gcode.error(
                "Failed calibration - didn't find %d unique steps" % (
                    full_steps,))
        min_step = angles_to_step[min(angles_to_step)]
        inc_angles = []
        for i in range(full_steps):
            inc_angles.append(angles[(i + min_step) % full_steps])
        # Correct the list if the wiring/encoder is inverted
        reversed_cal = False
        if inc_angles[1] > inc_angles[2]:
            reversed_cal = True
            inc_angles = inc_angles[1:] + inc_angles[:1]
            inc_angles.reverse()
        msg = "mechaduino calibration: Stddev=%.3f (%d of %d queries)" % (
            math.sqrt(total_variance / total_count), total_count,
            full_steps * 10)
        logging.info(msg)
        self.gcode.respond_info(msg)
        # Save results
        cal_contents = []
        for i, angle in enumerate(inc_angles):
            if not i % 8:
                cal_contents.append('\n')
            cal_contents.append("%.1f" % (angle,))
            cal_contents.append(',')
        cal_contents.pop()
        configfile = self.printer.lookup_object('configfile')
        configfile.remove_section(self.name)
        configfile.set(self.name, 'calibrate', ''.join(cal_contents))
        configfile.set(self.name, 'reversed_cal', reversed_cal)
        configfile.set(self.name, 'calibrate_start_step', '%d' % (min_step,))


######################################################################
# Main wrapper
######################################################################

class PrinterMechaduino:
    def __init__(self, config):
        self.printer = config.get_printer()
        servo_name = config.get_name().split()[1]
        self.a4954 = MCU_a4954(config)
        step_dist = config.getfloat('step_distance', above=0.)
        invert_dir = config.getboolean('invert_direction', False)
        self.mcu_vstepper = MCU_virtual_stepper(
            self.a4954.get_mcu(), servo_name, invert_dir, step_dist)
        force_move = self.printer.try_load_module(config, 'force_move')
        force_move.register_stepper(self.mcu_vstepper)
        self.servo_stepper = MCU_servo_stepper(config, self.a4954,
                                               self.mcu_vstepper)
        self.spi_position = MCU_spi_position(config, self.servo_stepper)
        ServoCalibration(config, self.mcu_vstepper, self.servo_stepper,
                         self.spi_position)
        stepper_enable = self.printer.try_load_module(config, 'stepper_enable')
        stepper_enable.register_servo_stepper(self.mcu_vstepper, self)
        # Register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command("SET_TORQUE_MODE", "SERVO", servo_name,
                                        self.cmd_SET_TORQUE_MODE,
                                        desc=self.cmd_SET_TORQUE_MODE_help)

        # XXX - Temp commands for Testing, Will Remove
        self.gcode.register_mux_command(
            "GET_SERVO_STATS", "SERVO", servo_name,
            self.cmd_GET_SERVO_STATS,
            desc=self.cmd_GET_SERVO_STATS_help)

    def get_mcu_stepper(self):
        return self.mcu_vstepper
    def set_enable(self, print_time):
        mode = self.servo_stepper.get_servo_mode()
        if mode == "open_loop":
            self.servo_stepper.set_open_loop_mode(print_time)
        elif mode == "hpid":
            self.servo_stepper.set_hpid_mode(print_time)
    def set_disable(self, print_time):
        self.servo_stepper.set_disabled(print_time)
    cmd_SET_TORQUE_MODE_help = "Place servo in torque mode"
    def cmd_SET_TORQUE_MODE(self, params):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        excite = self.gcode.get_int("EXCITE", params, 64)
        current = self.gcode.get_float("CURRENT", params)
        if not current:
            self.servo_stepper.set_disabled(print_time)
        else:
            self.servo_stepper.set_torque_mode(print_time, excite, current)
    cmd_GET_SERVO_STATS_help = "Retreive various sevo stepper statistics"
    def cmd_GET_SERVO_STATS(self, params):
        realtime_pos = self.mcu_vstepper.get_realtime_position()
        enc_pos, intp_ticks = self.spi_position.get_position()
        servo_err, pid_ticks = self.servo_stepper.get_servo_stats()
        msg = "Stepper Position: %d\nEncoder Position: %d\nServo Error: %d" % \
              (realtime_pos, enc_pos, servo_err)
        clock_mhz = self.servo_stepper.mcu_freq / 1000000.
        if intp_ticks is not None:
            intp_us = intp_ticks / clock_mhz
            msg += "\nLast Interpolation Time: %.2f us" % (intp_us)
        if pid_ticks is not None:
            pid_us = pid_ticks / clock_mhz
            msg += "\nMax PID Loop Time: %.2f us" % (pid_us)
        self.gcode.respond_info(msg)

def load_config_prefix(config):
    return PrinterMechaduino(config)
