# Save the current offset values set by the SET_GCODE_OFFSET command in printer config
#
# Copyright (C) 2019-2020 Bob Houston <bob@deltamaker.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# File: ~/klipper/klippy/extras/save_gcode_offset.py
#

class GCodeOffset:
    home_when_ready = False
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_offset = [0.0, 0.0, 0.0]
        try:
            self.gcode_offset = [float(x) for x in config.get("gcode_offset", default=",,").split(",")]
        except:
            self.gcode.respond_info("Initializing gcode_offset in %s" % (self.name))

        self.printer.register_event_handler("klippy:ready", self.restore_gcode_offset)
        self.gcode.register_command("SAVE_GCODE_OFFSET", self.cmd_SAVE_GCODE_OFFSET,
            desc=self.cmd_SAVE_GCODE_OFFSET_help)

    def restore_gcode_offset(self):
        self.gcode.respond_info(
            	"restore_gcode_offset: X=%.3f Y=%.3f Z=%.3f" % tuple(self.gcode_offset))
        self.gcode.run_script(
            "SET_GCODE_OFFSET X=%.3f Y=%.3f Z=%.3f" % tuple(self.gcode_offset))
        # Home the toolhead if requested
        if GCodeOffset.home_when_ready:
            # homing when ready stopped working after recent git pull
            GCodeOffset.home_when_ready = False
            self.gcode.run_script("G28")

    cmd_SAVE_GCODE_OFFSET_help = "store g-code offset positions in printer config"
    def cmd_SAVE_GCODE_OFFSET(self, gcmd):
        # Assumption: we can get the current offset values from the GCodeMove object.
        # Need to confirm the homing_position array is a reliable source for these values. 
        gcode_move = self.printer.lookup_object('gcode_move')
        self.gcode_offset = gcode_move.homing_position[:3]

        self.gcode.respond_info(
            "gcode_offset: %.3f, %.3f, %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % tuple(self.gcode_offset))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'gcode_offset', "%.3f, %.3f, %.3f" % tuple(self.gcode_offset))
        GCodeOffset.home_when_ready = (gcmd.get_int('HOME', 0) != 0)

def load_config(config):
    return GCodeOffset(config)
