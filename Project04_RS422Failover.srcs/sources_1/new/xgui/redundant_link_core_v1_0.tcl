# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "ALERT_HOLD_CYCLES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "BAUD_RATE" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CHANNEL_TIMEOUT_CYCLES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CLK_FREQ_HZ" -parent ${Page_0}
  ipgui::add_param $IPINST -name "EVENT_FIFO_DEPTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "FRAME_TIMEOUT_CLKS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "HEARTBEAT_CYCLES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "HISTORY_DEPTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "INTERBYTE_TIMEOUT_CLKS" -parent ${Page_0}
  ipgui::add_param $IPINST -name "PAIR_TIMEOUT_CYCLES" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SCAN_TICK_CYCLES" -parent ${Page_0}


}

proc update_PARAM_VALUE.ALERT_HOLD_CYCLES { PARAM_VALUE.ALERT_HOLD_CYCLES } {
	# Procedure called to update ALERT_HOLD_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.ALERT_HOLD_CYCLES { PARAM_VALUE.ALERT_HOLD_CYCLES } {
	# Procedure called to validate ALERT_HOLD_CYCLES
	return true
}

proc update_PARAM_VALUE.BAUD_RATE { PARAM_VALUE.BAUD_RATE } {
	# Procedure called to update BAUD_RATE when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.BAUD_RATE { PARAM_VALUE.BAUD_RATE } {
	# Procedure called to validate BAUD_RATE
	return true
}

proc update_PARAM_VALUE.CHANNEL_TIMEOUT_CYCLES { PARAM_VALUE.CHANNEL_TIMEOUT_CYCLES } {
	# Procedure called to update CHANNEL_TIMEOUT_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CHANNEL_TIMEOUT_CYCLES { PARAM_VALUE.CHANNEL_TIMEOUT_CYCLES } {
	# Procedure called to validate CHANNEL_TIMEOUT_CYCLES
	return true
}

proc update_PARAM_VALUE.CLK_FREQ_HZ { PARAM_VALUE.CLK_FREQ_HZ } {
	# Procedure called to update CLK_FREQ_HZ when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CLK_FREQ_HZ { PARAM_VALUE.CLK_FREQ_HZ } {
	# Procedure called to validate CLK_FREQ_HZ
	return true
}

proc update_PARAM_VALUE.EVENT_FIFO_DEPTH { PARAM_VALUE.EVENT_FIFO_DEPTH } {
	# Procedure called to update EVENT_FIFO_DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.EVENT_FIFO_DEPTH { PARAM_VALUE.EVENT_FIFO_DEPTH } {
	# Procedure called to validate EVENT_FIFO_DEPTH
	return true
}

proc update_PARAM_VALUE.FRAME_TIMEOUT_CLKS { PARAM_VALUE.FRAME_TIMEOUT_CLKS } {
	# Procedure called to update FRAME_TIMEOUT_CLKS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.FRAME_TIMEOUT_CLKS { PARAM_VALUE.FRAME_TIMEOUT_CLKS } {
	# Procedure called to validate FRAME_TIMEOUT_CLKS
	return true
}

proc update_PARAM_VALUE.HEARTBEAT_CYCLES { PARAM_VALUE.HEARTBEAT_CYCLES } {
	# Procedure called to update HEARTBEAT_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.HEARTBEAT_CYCLES { PARAM_VALUE.HEARTBEAT_CYCLES } {
	# Procedure called to validate HEARTBEAT_CYCLES
	return true
}

proc update_PARAM_VALUE.HISTORY_DEPTH { PARAM_VALUE.HISTORY_DEPTH } {
	# Procedure called to update HISTORY_DEPTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.HISTORY_DEPTH { PARAM_VALUE.HISTORY_DEPTH } {
	# Procedure called to validate HISTORY_DEPTH
	return true
}

proc update_PARAM_VALUE.INTERBYTE_TIMEOUT_CLKS { PARAM_VALUE.INTERBYTE_TIMEOUT_CLKS } {
	# Procedure called to update INTERBYTE_TIMEOUT_CLKS when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.INTERBYTE_TIMEOUT_CLKS { PARAM_VALUE.INTERBYTE_TIMEOUT_CLKS } {
	# Procedure called to validate INTERBYTE_TIMEOUT_CLKS
	return true
}

proc update_PARAM_VALUE.PAIR_TIMEOUT_CYCLES { PARAM_VALUE.PAIR_TIMEOUT_CYCLES } {
	# Procedure called to update PAIR_TIMEOUT_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.PAIR_TIMEOUT_CYCLES { PARAM_VALUE.PAIR_TIMEOUT_CYCLES } {
	# Procedure called to validate PAIR_TIMEOUT_CYCLES
	return true
}

proc update_PARAM_VALUE.SCAN_TICK_CYCLES { PARAM_VALUE.SCAN_TICK_CYCLES } {
	# Procedure called to update SCAN_TICK_CYCLES when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SCAN_TICK_CYCLES { PARAM_VALUE.SCAN_TICK_CYCLES } {
	# Procedure called to validate SCAN_TICK_CYCLES
	return true
}


proc update_MODELPARAM_VALUE.CLK_FREQ_HZ { MODELPARAM_VALUE.CLK_FREQ_HZ PARAM_VALUE.CLK_FREQ_HZ } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CLK_FREQ_HZ}] ${MODELPARAM_VALUE.CLK_FREQ_HZ}
}

proc update_MODELPARAM_VALUE.BAUD_RATE { MODELPARAM_VALUE.BAUD_RATE PARAM_VALUE.BAUD_RATE } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.BAUD_RATE}] ${MODELPARAM_VALUE.BAUD_RATE}
}

proc update_MODELPARAM_VALUE.INTERBYTE_TIMEOUT_CLKS { MODELPARAM_VALUE.INTERBYTE_TIMEOUT_CLKS PARAM_VALUE.INTERBYTE_TIMEOUT_CLKS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.INTERBYTE_TIMEOUT_CLKS}] ${MODELPARAM_VALUE.INTERBYTE_TIMEOUT_CLKS}
}

proc update_MODELPARAM_VALUE.FRAME_TIMEOUT_CLKS { MODELPARAM_VALUE.FRAME_TIMEOUT_CLKS PARAM_VALUE.FRAME_TIMEOUT_CLKS } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.FRAME_TIMEOUT_CLKS}] ${MODELPARAM_VALUE.FRAME_TIMEOUT_CLKS}
}

proc update_MODELPARAM_VALUE.PAIR_TIMEOUT_CYCLES { MODELPARAM_VALUE.PAIR_TIMEOUT_CYCLES PARAM_VALUE.PAIR_TIMEOUT_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PAIR_TIMEOUT_CYCLES}] ${MODELPARAM_VALUE.PAIR_TIMEOUT_CYCLES}
}

proc update_MODELPARAM_VALUE.CHANNEL_TIMEOUT_CYCLES { MODELPARAM_VALUE.CHANNEL_TIMEOUT_CYCLES PARAM_VALUE.CHANNEL_TIMEOUT_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CHANNEL_TIMEOUT_CYCLES}] ${MODELPARAM_VALUE.CHANNEL_TIMEOUT_CYCLES}
}

proc update_MODELPARAM_VALUE.EVENT_FIFO_DEPTH { MODELPARAM_VALUE.EVENT_FIFO_DEPTH PARAM_VALUE.EVENT_FIFO_DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.EVENT_FIFO_DEPTH}] ${MODELPARAM_VALUE.EVENT_FIFO_DEPTH}
}

proc update_MODELPARAM_VALUE.HISTORY_DEPTH { MODELPARAM_VALUE.HISTORY_DEPTH PARAM_VALUE.HISTORY_DEPTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.HISTORY_DEPTH}] ${MODELPARAM_VALUE.HISTORY_DEPTH}
}

proc update_MODELPARAM_VALUE.SCAN_TICK_CYCLES { MODELPARAM_VALUE.SCAN_TICK_CYCLES PARAM_VALUE.SCAN_TICK_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SCAN_TICK_CYCLES}] ${MODELPARAM_VALUE.SCAN_TICK_CYCLES}
}

proc update_MODELPARAM_VALUE.HEARTBEAT_CYCLES { MODELPARAM_VALUE.HEARTBEAT_CYCLES PARAM_VALUE.HEARTBEAT_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.HEARTBEAT_CYCLES}] ${MODELPARAM_VALUE.HEARTBEAT_CYCLES}
}

proc update_MODELPARAM_VALUE.ALERT_HOLD_CYCLES { MODELPARAM_VALUE.ALERT_HOLD_CYCLES PARAM_VALUE.ALERT_HOLD_CYCLES } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.ALERT_HOLD_CYCLES}] ${MODELPARAM_VALUE.ALERT_HOLD_CYCLES}
}
