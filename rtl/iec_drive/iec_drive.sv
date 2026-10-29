//-------------------------------------------------------------------------------
//
// C1541/C1581 selector
// (C) 2021 Alexey Melnikov
//
//-------------------------------------------------------------------------------
 
module iec_drive #(parameter PARPORT=1,DUALROM=1,DRIVES=2,
                  // 0 = the c1541/c1581 engines this core has always run.
                  // 1 = the vendored c157x engine, which adds the 1570/1571.
                  // Default 0 until the 157x path has run on hardware: the
                  // generate below synthesises exactly one of them, so with
                  // this at 0 nothing about the working drives changes.
                  USE_C157X=1)
(
	//clk ports
	input         clk,
	input   [N:0] reset,
	input         ce,

	input         pause,

	input   [N:0] img_mounted,
	input         img_readonly,
	input  [31:0] img_size,
	
	// THREE bits, expanded by main.vhd from the framework two. vdrives.vhd stays
	// 2-bit so M2M/ remains core-independent; code 01 there was G64, which the
	// Shell never sends, and main.vhd maps it to 100 here. doc/drive_models.md 4.7.
	// 000 - 1541 emulated GCR (D64)
	// 010 - 1581 (D81)
	// 011 - CMD HD
	// 100 - 1571 (D71), once c157x_multi is instantiated
	input   [2:0] img_type,

	// MEGA65 physical internal 1581 (issues #90 and #93): per-drive mode select.
	// Bit i set = drive i is backed by the real internal 3.5" drive: its 1581
	// engine is forced active, its 1541 engine is held in reset and its image
	// DMA (sd_rd/sd_wr) is masked off. There is only ONE physical mechanism, so
	// at most one bit may be set at a time (the caller enforces this); the
	// shared phys_* bundle below is driven by that drive (see c1581_multi).
	input   [N:0] physical_mode,

	output  [N:0] led,

	input         iec_atn_i,
	input         iec_data_i,
	input         iec_clk_i,
	output        iec_data_o,
	output        iec_clk_o,

	// parallel bus
	input   [7:0] par_data_i,
	input         par_stb_i,
	output  [7:0] par_data_o,
	output        par_stb_o,

	//clk_sys ports
	input         clk_sys,

	output reg [31:0] sd_lba[NDR],
	output reg  [5:0] sd_blk_cnt[NDR],
	output reg  [N:0] sd_rd,
	output reg  [N:0] sd_wr,
	input   [N:0] sd_ack,
	input  [13:0] sd_buff_addr,

	// External per-drive DOS ROM store, only used when USE_C157X=1. c157x_multi
	// holds no ROM of its own: it presents one address per drive and needs every
	// one answered in the same cycle. Flat vectors because the store that serves
	// them (CORE/vhdl/drive_rom_157x.vhd) is VHDL. See doc/drive_models.md 4.5.
	output [15*DRIVES-1:0] rom157x_addr_o,
	input  [ 8*DRIVES-1:0] rom157x_data_i,
	input   [7:0] sd_buff_dout,
	output reg [7:0] sd_buff_din[NDR],
	input         sd_buff_wr,

	// MEGA65: [17:15] is a ROM SLOT number and [14:0] the address within that slot. It began
	// as a single bit (0 = the 1541 custom DOS, 1 = the 1581 one) and grew into a field when
	// the drive models stopped sharing one ROM each. The slots are assigned in the QNICE
	// device decode in mega65.vhd, one C_DEV_C64_* device each:
	//    0 = 1541 custom DOS (JiffyDOS), 16 KB   2 = stock 1541-II DOS, 16 KB
	//    1 = 1581 custom DOS (JiffyDOS), 32 KB   3 + drive = that drives own slot, 16 KB
	input  [17:0] rom_addr_i,
	input   [7:0] rom_data_i,
	output reg [7:0] rom_data_o,
	input         rom_wr_i,
	// MEGA65: one bit per drive, and one vector per ENGINE. 1 = that drive runs the factory
	// DOS, 0 = the custom (JiffyDOS) one. This was a single bit for the whole chip, wired to
	// the C64 KERNAL choice, so the drives could be set neither independently of the computer
	// nor of each other.
	//
	// Two vectors because the two engines have two separate custom ROMs, loaded from two
	// different files (jd-c1541.bin and jd-c1581.bin) into two different QNICE devices. The
	// caller gates each one on ITS OWN file having turned up, so asking for JiffyDOS on a
	// drive whose engine has no custom ROM leaves that drive on the factory DOS instead of
	// running it out of blank memory.
	input   [N:0] rom_std_i,
	input   [N:0] rom_std_1581_i,

	// MEGA65: which MODEL each drive on the factory DOS is. 1 = stock 1541-II, 0 = stock 1541.
	// Only consulted where rom_std_i is 1 -- JiffyDOS replaces the DOS outright, so it wins on
	// either model. The 1581 has no second model, hence no twin vector for it.
	input   [N:0] rom_ii_i,

	// MEGA65: 1 = this drive runs the ROM file the user loaded into its own slot, which beats
	// both of the vectors above. Gated by the caller on a file having arrived.
	input   [N:0] rom_own_i,

	// ---------------------------------------------------------------------
	// MEGA65 physical internal 1581 (issue #90): drive-0 flat toggle/level ABI
	// to the VHDL physical_1581_controller (50 MHz, instantiated in main.vhd).
	// Threaded straight through c1581_multi -> drive-0 c1581_drv -> fdc1772.
	// Inert unless physical_mode=1. main.vhd connects these ports.
	// ---------------------------------------------------------------------
	// phys OUTPUTS (fdc1772 -> controller)
	output        phys_active,
	output        phys_cia_motor_on,
	output        phys_cia_side,
	output        phys_step_req_tgl,
	output        phys_step_outward,
	output        phys_rd_req_tgl,
	output  [2:0] phys_rd_op,
	output  [7:0] phys_rd_track,
	output        phys_rd_side,
	output  [7:0] phys_rd_sector,
	output        phys_rd_cancel_tgl,
	output  [1:0] phys_rd_seq,       // op sequence tag (quasi-static before rd_req_tgl)
	output        phys_byte_ovf,
	output        phys_byte_rd_en,

	// phys INPUTS (controller -> fdc1772)
	input         phys_step_ack_tgl,
	input         phys_rd_done_tgl,
	input   [1:0] phys_rd_done_seq,  // seq of the op being completed (quasi-static before rd_done_tgl)
	input   [4:0] phys_rd_result,
	input         phys_rd_crc_err,
	input         phys_rd_rnf,
	input         phys_rd_deleted,
	input   [7:0] phys_rd_c,
	input   [7:0] phys_rd_h,
	input   [7:0] phys_rd_r,
	input   [7:0] phys_rd_n,
	input   [7:0] phys_byte_data,
	input         phys_byte_empty,
	input         phys_media_ready,
	input         phys_index,
	input         phys_track0,
	input         phys_wprot,
	input         phys_change,
	input         phys_motor_on,
	input         phys_head_settled,

	// fdc1772 diagnostic event toggles + per-op presented-byte count (drive
	// clock; main.vhd 2-FF-syncs the toggles into the 50 MHz diag domain,
	// pres_cnt crosses unsynced as a quasi-static bus captured on the fin edge)
	output        phys_dbg_lost_tgl,
	output        phys_dbg_drain_tgl,
	output        phys_dbg_staledone_tgl,
	output        phys_dbg_busycmd_tgl,
	output        phys_dbg_fin_tgl,
	output [10:0] phys_dbg_pres_cnt
);

localparam NDR = (DRIVES < 1) ? 1 : (DRIVES > 4) ? 4 : DRIVES;
localparam N   = NDR - 1;

// MEGA65 (#90/#93): per-drive physical-mode vector (at most one bit set, see
// the port comment). Each bit affects only its drive's engine-select and
// sd_rd/sd_wr gating.
wire [N:0] phys_mode_vec = physical_mode;

// THREE bit-slices, not two. The fourth state is the 1571: img_type 100 cannot
// be held in two bits, and dtype is what every engine-enable below is built on.
//   000 1541   010 1581   011 CMD HD   100 1571
reg [N:0] dtype[3];
wire        c1541_iec_data, c1541_iec_clk, c1541_stb_o;
wire  [7:0] c1541_par_o;
wire  [N:0] c1541_led;
wire  [7:0] c1541_sd_buff_dout[NDR];
wire [31:0] c1541_sd_lba[NDR];
wire  [N:0] c1541_sd_rd, c1541_sd_wr;
wire  [5:0] c1541_sd_blk_cnt[NDR];

wire        c1581_iec_data, c1581_iec_clk, c1581_stb_o;
wire  [7:0] c1581_par_o;
wire  [N:0] c1581_led;
wire  [7:0] c1581_sd_buff_dout[NDR];
wire [31:0] c1581_sd_lba[NDR];
wire  [N:0] c1581_sd_rd, c1581_sd_wr;

// MEGA65 (D81 enable, sy2002): re-homed from clk_sys to clk. Upstream MiSTer sources
// img_mounted/img_size/img_type from hps_io on clk_sys, so latching on posedge clk_sys was
// same-domain. In the M2M port these three come from vdrives, which resynchronizes them into
// the CORE clock domain (clk = clk_main_i). Sampling them on the QNICE clock (clk_sys) was an
// unsynchronized CDC: D64 survived it only because dtype powers up to 0 (= 1541), but a D81
// (img_type=10) needs this latch to actually capture a non-zero value across the boundary.
// Latch in the signals own domain instead. dtype is quasi-static (changes only on a mount or
// unmount strobe), so its use as the clk_sys-domain sd_lba/sd_rd mux select stays safe.
//
// MEGA65 (#88/#93): an unmount strobe (img_mounted with img_size = 0, sent by the Shell when
// the user unmounts and by vdrives after a reset-driven unmount) returns the drive to 00 =
// 1541, the power-up value. Upstream only latched on mounts, so dtype kept the last mounted
// type: with "Disk Image: Always" (the drive keeps running while unmounted) a drive that once
// had a D81 mounted stayed an empty 1581 after the unmount, and programs that upload code into
// the 1541 (the 4k intro "Boo" by Reflex, issue #88) found no 1541 until a D64 was mounted
// again. The 1581 engine is live only while a D81 is mounted or in physical mode. The latch
// stays level-sensitive: the strobe is held for many clk cycles while img_size/img_type
// arrive through separate synchronizers, so the last strobe cycle wins with settled values.
always @(posedge clk) for(int i=0; i<NDR; i=i+1) if(img_mounted[i]) {dtype[2][i],dtype[1][i],dtype[0][i]} <= (|img_size) ? img_type : 3'b000;

// This drive is a 1571, which only the c157x engine can be. Kept as its own name
// because it gates an engine, a drv_mode and a double-sided flag, and spelling
// dtype[2] at each of them is how an index goes wrong.
wire [N:0] is_157x = dtype[2];

// c157x_multi takes unpacked arrays; the ROM store across the VHDL boundary takes
// flat vectors. Convert here rather than at the port, so the instantiation reads
// like the upstream one and the packing is in one place.
wire  [1:0] drv_mode[NDR];
wire [14:0] mem_a_arr[NDR];
wire  [7:0] rom_do_arr[NDR];

// 10 is the 1571, 00 the 1541. The 1570 (01) and the 1571CR (11) are not offered:
// the CR is marked todo upstream with its RAM commented out.
for (genvar gi = 0; gi < NDR; gi = gi + 1) begin : g_drvmode
	assign drv_mode[gi]  = is_157x[gi] ? 2'b10 : 2'b00;
	assign rom_do_arr[gi] = rom157x_data_i[gi*8 + 7 -: 8];
end

assign led          = c1581_led      | c1541_led;     // MEGA65 (D81): 1581 engine enabled
assign iec_data_o   = c1581_iec_data & c1541_iec_data;
assign iec_clk_o    = c1581_iec_clk  & c1541_iec_clk;
assign par_stb_o    = c1581_stb_o    & c1541_stb_o;
assign par_data_o   = c1581_par_o    & c1541_par_o;

// MEGA65 (D81): ROM readback mux, one arm per slot of rom_addr_i[17:15]. The ROM slots are
// FALLING_A (QNICE writes/reads on the falling edge of clk_sys), so the slot number is
// captured on the SAME falling edge to stay aligned with the q_a read data. (Readback only --
// the auto-loader never reads ROMs back; this is forward-proofing for any future
// verify-after-write of a streamed drive DOS image.)
wire [2:0] rom_slot = rom_addr_i[17:15];
wire       rom_slot_own = (rom_slot >= 3'd3) && (rom_slot <= 3'd6);  // slot 3 + drive
wire [1:0] rom_own_sel  = rom_slot - 3'd3;             // slot 3..6 -> drive 0..3
wire [7:0] c1541_rom_data_o, c1581_rom_data_o, c1541ii_rom_data_o, c1541own_rom_data_o;
reg  [2:0] rom_sel_d;
always @(negedge clk_sys) rom_sel_d <= rom_slot;
always_comb begin
	case(rom_sel_d)
		3'd1:    rom_data_o = c1581_rom_data_o;
		3'd2:    rom_data_o = c1541ii_rom_data_o;
		3'd0:    rom_data_o = c1541_rom_data_o;
		default: rom_data_o = c1541own_rom_data_o;
	endcase
end

always_comb for(int i=0; i<NDR; i=i+1) begin
	sd_buff_din[i] = (dtype[1][i] ? c1581_sd_buff_dout[i] : c1541_sd_buff_dout[i] );
	sd_lba[i]      = (dtype[1][i] ? c1581_sd_lba[i] << 1  : c1541_sd_lba[i]       );
	// MEGA65 (#90): in physical mode the internal drive is backed by the real
	// disk, so mask its image DMA off. sd_lba/sd_blk_cnt/sd_buff_din are left as
	// is -- harmless while sd_rd=sd_wr=0 (vdrives only acts on rd|wr).
	sd_rd[i]       = phys_mode_vec[i] ? 1'b0 : (dtype[1][i] ? c1581_sd_rd[i] : c1541_sd_rd[i]);
	sd_wr[i]       = phys_mode_vec[i] ? 1'b0 : (dtype[1][i] ? c1581_sd_wr[i] : c1541_sd_wr[i]);
	sd_blk_cnt[i]  = (dtype[1][i] ? 6'd1                  : c1541_sd_blk_cnt[i]   );
end

generate
if (!USE_C157X) begin : g_c1541

c1541_multi #(.PARPORT(PARPORT), .DUALROM(DUALROM), .DRIVES(DRIVES)) c1541
(
	.clk(clk),
	// MEGA65 (#90): also hold the 1541 engine in reset in physical mode -- only
	// one engine may own the AND-wired IEC bus, and physical mode is 1581-only.
	.reset(reset | dtype[1] | phys_mode_vec),
	.ce(ce),

	.gcr_mode(dtype[0]),

	.iec_atn_i (iec_atn_i),
	.iec_data_i(iec_data_i & c1581_iec_data),
	.iec_clk_i (iec_clk_i  & c1581_iec_clk),
	.iec_data_o(c1541_iec_data),
	.iec_clk_o (c1541_iec_clk),

	.led(c1541_led),

	.par_data_i(par_data_i),
	.par_stb_i(par_stb_i),
	.par_data_o(c1541_par_o),
	.par_stb_o(c1541_stb_o),

	.clk_sys(clk_sys),
	.pause(pause),

	.rom_addr_i(rom_addr_i[14:0]),
	.rom_data_i(rom_data_i),
	.rom_data_o(c1541_rom_data_o),
	.rom_wr_i(rom_wr_i & (rom_slot == 3'd0)),
	.rom_ii_wr_i(rom_wr_i & (rom_slot == 3'd2)),
	.rom_ii_data_o(c1541ii_rom_data_o),
	.rom_own_sel_i(rom_own_sel),
	.rom_own_wr_i(rom_wr_i & rom_slot_own),
	.rom_own_data_o(c1541own_rom_data_o),
	.rom_std_i(rom_std_i),
	.rom_ii_i(rom_ii_i),
	.rom_own_i(rom_own_i),

	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.sd_lba(c1541_sd_lba),
	.sd_blk_cnt(c1541_sd_blk_cnt),
	.sd_rd(c1541_sd_rd),
	.sd_wr(c1541_sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(c1541_sd_buff_dout),
	.sd_buff_wr(sd_buff_wr)
);

// Nothing reads the store in this branch, so give it a defined value rather than
// leaving an output floating.
assign rom157x_addr_o = '0;

end else begin : g_c157x

// The vendored 1541/1570/1571 engine. Differences from the block above that are
// NOT cosmetic, each from doc/drive_models.md:
//
//   drv_mode   per drive, 00 = 1541 and 10 = 1571. 01 is the 1570 and 11 the
//              1571CR, which upstream marks todo and whose RAM is commented
//              out, so neither is offered (4.2).
//   img_mfm    TIED LOW. MFM on a 1571 reads CP/M and MS-DOS disks, which is
//              not what a C64 core is for, and dropping it takes
//              TRACK_BUF_LEN to 8192 so per-drive block RAM stays as it is (4.2).
//   iec_fclk   left open. A real C64 has no fast serial -- CIA1 SP and CNT are
//              not on the IEC connector, which is exactly why a 1571 runs in
//              1541 mode on a C64 -- so burst mode legitimately cannot work
//              here and pretending otherwise would be worse (4.3).
//   ROM        external, one block RAM per drive, addressed through
//              rom157x_addr_o / rom157x_data_i (4.5).
//   sd_buff_addr is 16 bits here against our 14, so it is zero-extended. That
//              caps the transfer buffer at 16 kB, which is far more than a
//              block transfer uses.
c157x_multi #(.PARPORT(PARPORT), .DRIVES(DRIVES)) c157x
(
	.clk(clk),
	.reset(reset | dtype[1] | phys_mode_vec),
	.ce(ce),
	.pause(pause),

	.drv_mode(drv_mode),

	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),
	.img_ds(is_157x),
	.img_gcr(~is_157x),
	.img_mfm({NDR{1'b0}}),

	.led(c1541_led),
	.disk_ready(),
	.out_track(),
	.out_we(),

	.iec_atn_i (iec_atn_i),
	.iec_data_i(iec_data_i & c1581_iec_data),
	.iec_clk_i (iec_clk_i  & c1581_iec_clk),
	.iec_fclk_i(1'b1),
	.iec_data_o(c1541_iec_data),
	.iec_clk_o (c1541_iec_clk),
	.iec_fclk_o(),

	.par_data_i(par_data_i),
	.par_stb_i(par_stb_i),
	.par_data_o(c1541_par_o),
	.par_stb_o(c1541_stb_o),

	.clk_sys(clk_sys),

	.sd_lba(c1541_sd_lba),
	.sd_blk_cnt(c1541_sd_blk_cnt),
	.sd_rd(c1541_sd_rd),
	.sd_wr(c1541_sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr({2'b00, sd_buff_addr}),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(c1541_sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),

	.mem_a(mem_a_arr),
	.rom_do(rom_do_arr),
	.empty8k({NDR{1'b0}})
);

for (genvar gj = 0; gj < NDR; gj = gj + 1) begin : g_rompack
	assign rom157x_addr_o[gj*15 + 14 -: 15] = mem_a_arr[gj];
end

end
endgenerate


// MEGA65 (D81 enable, sy2002): the 1581 engine is now active. Reset is released only while
// the drive has a D81 mounted (dtype[1]=1); a D64 or no image at all holds it in reset so only
// one engine drives the IEC bus at a time (AND-wired, safe by construction -- a reset drive
// contributes '1').
// The stale signal names in the original commented block (rom_addr/rom_data/rom_wr/rom_std)
// are corrected to the actual _i-suffixed ports; slot 1 of rom_addr_i[17:15] is the 1581 ROM
// window. iec_fclk_o and pwr_led are intentionally left unconnected (C64 has no fast serial).
c1581_multi #(.PARPORT(PARPORT), .DUALROM(DUALROM), .DRIVES(DRIVES)) c1581
(
	.clk(clk),
	// MEGA65 (#90): release the 1581 engine when a D81 is mounted (dtype[1]=1)
	// OR when physical mode is selected -- physical mode has no D81 image but
	// must still run the 1581 engine (backed by the real internal drive).
	.reset(reset | ~(dtype[1] | phys_mode_vec)),
	.ce(ce),

	.iec_atn_i (iec_atn_i),
	.iec_data_i(iec_data_i & c1541_iec_data),
	.iec_clk_i (iec_clk_i  & c1541_iec_clk),
	.iec_fclk_i (1),
	.iec_data_o(c1581_iec_data),
	.iec_clk_o (c1581_iec_clk),

	.act_led(c1581_led),

	.par_data_i(par_data_i),
	.par_stb_i(par_stb_i),
	.par_data_o(c1581_par_o),
	.par_stb_o(c1581_stb_o),

	.clk_sys(clk_sys),
	.pause(pause),

	.rom_addr(rom_addr_i[14:0]),
	.rom_data(rom_data_i),
	.rom_data_o(c1581_rom_data_o),
	.rom_wr(rom_wr_i & (rom_slot == 3'd1)),
	.rom_std(rom_std_1581_i),

	.img_mounted(img_mounted),
	.img_size(img_size),
	.img_readonly(img_readonly),

	.sd_lba(c1581_sd_lba),
	.sd_rd(c1581_sd_rd),
	.sd_wr(c1581_sd_wr),
	.sd_ack(sd_ack),
	.sd_buff_addr(sd_buff_addr[8:0]),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(c1581_sd_buff_dout),
	.sd_buff_wr(sd_buff_wr),

	// MEGA65 (#90/#93): physical internal 1581 ABI, threaded to the one drive
	// whose phys_mode bit is set (onehot mux inside c1581_multi).
	.phys_mode(phys_mode_vec),
	.phys_active(phys_active),
	.phys_cia_motor_on(phys_cia_motor_on),
	.phys_cia_side(phys_cia_side),
	.phys_step_req_tgl(phys_step_req_tgl),
	.phys_step_outward(phys_step_outward),
	.phys_rd_req_tgl(phys_rd_req_tgl),
	.phys_rd_op(phys_rd_op),
	.phys_rd_track(phys_rd_track),
	.phys_rd_side(phys_rd_side),
	.phys_rd_sector(phys_rd_sector),
	.phys_rd_cancel_tgl(phys_rd_cancel_tgl),
	.phys_rd_seq(phys_rd_seq),
	.phys_byte_ovf(phys_byte_ovf),
	.phys_byte_rd_en(phys_byte_rd_en),
	.phys_step_ack_tgl(phys_step_ack_tgl),
	.phys_rd_done_tgl(phys_rd_done_tgl),
	.phys_rd_done_seq(phys_rd_done_seq),
	.phys_rd_result(phys_rd_result),
	.phys_rd_crc_err(phys_rd_crc_err),
	.phys_rd_rnf(phys_rd_rnf),
	.phys_rd_deleted(phys_rd_deleted),
	.phys_rd_c(phys_rd_c),
	.phys_rd_h(phys_rd_h),
	.phys_rd_r(phys_rd_r),
	.phys_rd_n(phys_rd_n),
	.phys_byte_data(phys_byte_data),
	.phys_byte_empty(phys_byte_empty),
	.phys_media_ready(phys_media_ready),
	.phys_index(phys_index),
	.phys_track0(phys_track0),
	.phys_wprot(phys_wprot),
	.phys_change(phys_change),
	.phys_motor_on(phys_motor_on),
	.phys_head_settled(phys_head_settled),
	.phys_dbg_lost_tgl(phys_dbg_lost_tgl),
	.phys_dbg_drain_tgl(phys_dbg_drain_tgl),
	.phys_dbg_staledone_tgl(phys_dbg_staledone_tgl),
	.phys_dbg_busycmd_tgl(phys_dbg_busycmd_tgl),
	.phys_dbg_fin_tgl(phys_dbg_fin_tgl),
	.phys_dbg_pres_cnt(phys_dbg_pres_cnt)
);
endmodule
