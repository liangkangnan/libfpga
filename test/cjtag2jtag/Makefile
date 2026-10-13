HDL := ../../../../hdl
.PHONY: all
all:
	iverilog -g2012 -o tb_cjtag2jtag.vvp -I . \
		tb_cjtag2jtag.v ../../utils/cjtag2jtag.v \
		$(HDL)/debug/dtm/hazard3_jtag_dtm.v \
		$(HDL)/debug/dtm/hazard3_jtag_dtm_core.v \
		$(HDL)/debug/cdc/hazard3_apb_async_bridge.v \
		$(HDL)/debug/cdc/hazard3_reset_sync.v \
		$(HDL)/debug/cdc/hazard3_sync_1bit.v
	vvp tb_cjtag2jtag.vvp
