# Save the current offset values set by the SET_GCODE_OFFSET command in printer config
#
# Copyright (C) 2019 Bob Houston <bob@deltamaker.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

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
            self.gcode.respond_info("Unable to parse gcode_offset in %s" % (self.name))

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_command("SAVE_GCODE_OFFSET", self.cmd_SAVE_GCODE_OFFSET, 
                                    desc=self.cmd_SAVE_GCODE_OFFSET_help)

    def home_command(self):
        self.gcode.run_script_from_command("G28")
        #cmd_G28 = self.gcode.register_command("G28", None)
        #self.gcode.register_command("G28", cmd_G28)
        #gcmd = self.gcode.create_gcode_command("G28", "G28", {})
        #cmd_G28(gcmd)

    def handle_ready(self):
        self.gcode.respond_info("gcode_offset: X=%.3f Y=%.3f Z=%.3f" % tuple(self.gcode_offset))
        self.gcode.run_script_from_command(
            "SET_GCODE_OFFSET X=%.3f Y=%.3f Z=%.3f" % tuple(self.gcode_offset))
        # Home the toolhead if requested
        if GCodeOffset.home_when_ready:
            GCodeOffset.home_when_ready = False
            home_command()
            # this doesn't work
            #gcmd = self.gcode.create_gcode_command("G28", "G28", {})
            #self.gcode.cmd_G28(gcmd)

    cmd_SAVE_GCODE_OFFSET_help = "store g-code offset positions in printer config"
    def cmd_SAVE_GCODE_OFFSET(self, gcmd):
        # Assumption: can get the current offset values from the GCodeParser object.
        # Need to confirm the homing_position array is a valid source for these values. 
        self.gcode_offset = self.gcode.homing_position[:3]

        gcmd.respond_info(
            "gcode_offset: %.3f, %.3f, %.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with the above and restart the printer." % tuple(self.gcode_offset))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(self.name, 'gcode_offset', "%.3f, %.3f, %.3f" % tuple(self.gcode_offset))
        GCodeOffset.home_when_ready = (gcmd.get_int('HOME', 0) > 0)

def load_config(config):
    return GCodeOffset(config)
