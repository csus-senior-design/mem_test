// LPDDR2x32_4p.v

// Generated using ACDS version 15.0 145

`timescale 1 ps / 1 ps
module LPDDR2x32_4p (
		input  wire        pll_ref_clk,                //        pll_ref_clk.clk
		input  wire        global_reset_n,             //       global_reset.reset_n
		input  wire        soft_reset_n,               //         soft_reset.reset_n
		output wire        afi_clk,                    //            afi_clk.clk
		output wire        afi_half_clk,               //       afi_half_clk.clk
		output wire        afi_reset_n,                //          afi_reset.reset_n
		output wire        afi_reset_export_n,         //   afi_reset_export.reset_n
		input  wire        seq_debug_clk,              //      seq_debug_clk.clk
		input  wire        seq_debug_reset_n,          // seq_debug_reset_in.reset_n
		output wire [9:0]  mem_ca,                     //             memory.mem_ca
		output wire [0:0]  mem_ck,                     //                   .mem_ck
		output wire [0:0]  mem_ck_n,                   //                   .mem_ck_n
		output wire [0:0]  mem_cke,                    //                   .mem_cke
		output wire [0:0]  mem_cs_n,                   //                   .mem_cs_n
		output wire [3:0]  mem_dm,                     //                   .mem_dm
		inout  wire [31:0] mem_dq,                     //                   .mem_dq
		inout  wire [3:0]  mem_dqs,                    //                   .mem_dqs
		inout  wire [3:0]  mem_dqs_n,                  //                   .mem_dqs_n
		output wire        avl_ready_0,                //              avl_0.waitrequest_n
		input  wire        avl_burstbegin_0,           //                   .beginbursttransfer
		input  wire [26:0] avl_addr_0,                 //                   .address
		output wire        avl_rdata_valid_0,          //                   .readdatavalid
		output wire [31:0] avl_rdata_0,                //                   .readdata
		input  wire [31:0] avl_wdata_0,                //                   .writedata
		input  wire [3:0]  avl_be_0,                   //                   .byteenable
		input  wire        avl_read_req_0,             //                   .read
		input  wire        avl_write_req_0,            //                   .write
		input  wire [2:0]  avl_size_0,                 //                   .burstcount
		output wire        avl_ready_1,                //              avl_1.waitrequest_n
		input  wire        avl_burstbegin_1,           //                   .beginbursttransfer
		input  wire [26:0] avl_addr_1,                 //                   .address
		output wire        avl_rdata_valid_1,          //                   .readdatavalid
		output wire [31:0] avl_rdata_1,                //                   .readdata
		input  wire [31:0] avl_wdata_1,                //                   .writedata
		input  wire [3:0]  avl_be_1,                   //                   .byteenable
		input  wire        avl_read_req_1,             //                   .read
		input  wire        avl_write_req_1,            //                   .write
		input  wire [2:0]  avl_size_1,                 //                   .burstcount
		output wire        avl_ready_2,                //              avl_2.waitrequest_n
		input  wire        avl_burstbegin_2,           //                   .beginbursttransfer
		input  wire [26:0] avl_addr_2,                 //                   .address
		output wire        avl_rdata_valid_2,          //                   .readdatavalid
		output wire [31:0] avl_rdata_2,                //                   .readdata
		input  wire [31:0] avl_wdata_2,                //                   .writedata
		input  wire [3:0]  avl_be_2,                   //                   .byteenable
		input  wire        avl_read_req_2,             //                   .read
		input  wire        avl_write_req_2,            //                   .write
		input  wire [2:0]  avl_size_2,                 //                   .burstcount
		output wire        avl_ready_3,                //              avl_3.waitrequest_n
		input  wire        avl_burstbegin_3,           //                   .beginbursttransfer
		input  wire [26:0] avl_addr_3,                 //                   .address
		output wire        avl_rdata_valid_3,          //                   .readdatavalid
		output wire [31:0] avl_rdata_3,                //                   .readdata
		input  wire [31:0] avl_wdata_3,                //                   .writedata
		input  wire [3:0]  avl_be_3,                   //                   .byteenable
		input  wire        avl_read_req_3,             //                   .read
		input  wire        avl_write_req_3,            //                   .write
		input  wire [2:0]  avl_size_3,                 //                   .burstcount
		input  wire        mp_cmd_clk_0_clk,           //       mp_cmd_clk_0.clk
		input  wire        mp_cmd_reset_n_0_reset_n,   //   mp_cmd_reset_n_0.reset_n
		input  wire        mp_cmd_clk_1_clk,           //       mp_cmd_clk_1.clk
		input  wire        mp_cmd_reset_n_1_reset_n,   //   mp_cmd_reset_n_1.reset_n
		input  wire        mp_cmd_clk_2_clk,           //       mp_cmd_clk_2.clk
		input  wire        mp_cmd_reset_n_2_reset_n,   //   mp_cmd_reset_n_2.reset_n
		input  wire        mp_cmd_clk_3_clk,           //       mp_cmd_clk_3.clk
		input  wire        mp_cmd_reset_n_3_reset_n,   //   mp_cmd_reset_n_3.reset_n
		input  wire        mp_rfifo_clk_0_clk,         //     mp_rfifo_clk_0.clk
		input  wire        mp_rfifo_reset_n_0_reset_n, // mp_rfifo_reset_n_0.reset_n
		input  wire        mp_wfifo_clk_0_clk,         //     mp_wfifo_clk_0.clk
		input  wire        mp_wfifo_reset_n_0_reset_n, // mp_wfifo_reset_n_0.reset_n
		input  wire        mp_rfifo_clk_1_clk,         //     mp_rfifo_clk_1.clk
		input  wire        mp_rfifo_reset_n_1_reset_n, // mp_rfifo_reset_n_1.reset_n
		input  wire        mp_wfifo_clk_1_clk,         //     mp_wfifo_clk_1.clk
		input  wire        mp_wfifo_reset_n_1_reset_n, // mp_wfifo_reset_n_1.reset_n
		input  wire        mp_rfifo_clk_2_clk,         //     mp_rfifo_clk_2.clk
		input  wire        mp_rfifo_reset_n_2_reset_n, // mp_rfifo_reset_n_2.reset_n
		input  wire        mp_wfifo_clk_2_clk,         //     mp_wfifo_clk_2.clk
		input  wire        mp_wfifo_reset_n_2_reset_n, // mp_wfifo_reset_n_2.reset_n
		input  wire        mp_rfifo_clk_3_clk,         //     mp_rfifo_clk_3.clk
		input  wire        mp_rfifo_reset_n_3_reset_n, // mp_rfifo_reset_n_3.reset_n
		input  wire        mp_wfifo_clk_3_clk,         //     mp_wfifo_clk_3.clk
		input  wire        mp_wfifo_reset_n_3_reset_n, // mp_wfifo_reset_n_3.reset_n
		output wire        local_init_done,            //             status.local_init_done
		output wire        local_cal_success,          //                   .local_cal_success
		output wire        local_cal_fail,             //                   .local_cal_fail
		input  wire        oct_rzqin,                  //                oct.rzqin
		output wire        pll_mem_clk,                //        pll_sharing.pll_mem_clk
		output wire        pll_write_clk,              //                   .pll_write_clk
		output wire        pll_locked,                 //                   .pll_locked
		output wire        pll_write_clk_pre_phy_clk,  //                   .pll_write_clk_pre_phy_clk
		output wire        pll_addr_cmd_clk,           //                   .pll_addr_cmd_clk
		output wire        pll_avl_clk,                //                   .pll_avl_clk
		output wire        pll_config_clk,             //                   .pll_config_clk
		output wire        pll_mem_phy_clk,            //                   .pll_mem_phy_clk
		output wire        afi_phy_clk,                //                   .afi_phy_clk
		output wire        pll_avl_phy_clk,            //                   .pll_avl_phy_clk
		input  wire [19:0] seq_debug_addr,             //          seq_debug.address
		input  wire        seq_debug_read_req,         //                   .read
		output wire [31:0] seq_debug_rdata,            //                   .readdata
		input  wire        seq_debug_write_req,        //                   .write
		input  wire [31:0] seq_debug_wdata,            //                   .writedata
		output wire        seq_debug_waitrequest,      //                   .waitrequest
		input  wire [3:0]  seq_debug_be,               //                   .byteenable
		output wire        seq_debug_rdata_valid       //                   .readdatavalid
	);

	LPDDR2x32_4p_0002 lpddr2x32_4p_inst (
		.pll_ref_clk                (pll_ref_clk),                //        pll_ref_clk.clk
		.global_reset_n             (global_reset_n),             //       global_reset.reset_n
		.soft_reset_n               (soft_reset_n),               //         soft_reset.reset_n
		.afi_clk                    (afi_clk),                    //            afi_clk.clk
		.afi_half_clk               (afi_half_clk),               //       afi_half_clk.clk
		.afi_reset_n                (afi_reset_n),                //          afi_reset.reset_n
		.afi_reset_export_n         (afi_reset_export_n),         //   afi_reset_export.reset_n
		.seq_debug_clk              (seq_debug_clk),              //      seq_debug_clk.clk
		.seq_debug_reset_n          (seq_debug_reset_n),          // seq_debug_reset_in.reset_n
		.mem_ca                     (mem_ca),                     //             memory.mem_ca
		.mem_ck                     (mem_ck),                     //                   .mem_ck
		.mem_ck_n                   (mem_ck_n),                   //                   .mem_ck_n
		.mem_cke                    (mem_cke),                    //                   .mem_cke
		.mem_cs_n                   (mem_cs_n),                   //                   .mem_cs_n
		.mem_dm                     (mem_dm),                     //                   .mem_dm
		.mem_dq                     (mem_dq),                     //                   .mem_dq
		.mem_dqs                    (mem_dqs),                    //                   .mem_dqs
		.mem_dqs_n                  (mem_dqs_n),                  //                   .mem_dqs_n
		.avl_ready_0                (avl_ready_0),                //              avl_0.waitrequest_n
		.avl_burstbegin_0           (avl_burstbegin_0),           //                   .beginbursttransfer
		.avl_addr_0                 (avl_addr_0),                 //                   .address
		.avl_rdata_valid_0          (avl_rdata_valid_0),          //                   .readdatavalid
		.avl_rdata_0                (avl_rdata_0),                //                   .readdata
		.avl_wdata_0                (avl_wdata_0),                //                   .writedata
		.avl_be_0                   (avl_be_0),                   //                   .byteenable
		.avl_read_req_0             (avl_read_req_0),             //                   .read
		.avl_write_req_0            (avl_write_req_0),            //                   .write
		.avl_size_0                 (avl_size_0),                 //                   .burstcount
		.avl_ready_1                (avl_ready_1),                //              avl_1.waitrequest_n
		.avl_burstbegin_1           (avl_burstbegin_1),           //                   .beginbursttransfer
		.avl_addr_1                 (avl_addr_1),                 //                   .address
		.avl_rdata_valid_1          (avl_rdata_valid_1),          //                   .readdatavalid
		.avl_rdata_1                (avl_rdata_1),                //                   .readdata
		.avl_wdata_1                (avl_wdata_1),                //                   .writedata
		.avl_be_1                   (avl_be_1),                   //                   .byteenable
		.avl_read_req_1             (avl_read_req_1),             //                   .read
		.avl_write_req_1            (avl_write_req_1),            //                   .write
		.avl_size_1                 (avl_size_1),                 //                   .burstcount
		.avl_ready_2                (avl_ready_2),                //              avl_2.waitrequest_n
		.avl_burstbegin_2           (avl_burstbegin_2),           //                   .beginbursttransfer
		.avl_addr_2                 (avl_addr_2),                 //                   .address
		.avl_rdata_valid_2          (avl_rdata_valid_2),          //                   .readdatavalid
		.avl_rdata_2                (avl_rdata_2),                //                   .readdata
		.avl_wdata_2                (avl_wdata_2),                //                   .writedata
		.avl_be_2                   (avl_be_2),                   //                   .byteenable
		.avl_read_req_2             (avl_read_req_2),             //                   .read
		.avl_write_req_2            (avl_write_req_2),            //                   .write
		.avl_size_2                 (avl_size_2),                 //                   .burstcount
		.avl_ready_3                (avl_ready_3),                //              avl_3.waitrequest_n
		.avl_burstbegin_3           (avl_burstbegin_3),           //                   .beginbursttransfer
		.avl_addr_3                 (avl_addr_3),                 //                   .address
		.avl_rdata_valid_3          (avl_rdata_valid_3),          //                   .readdatavalid
		.avl_rdata_3                (avl_rdata_3),                //                   .readdata
		.avl_wdata_3                (avl_wdata_3),                //                   .writedata
		.avl_be_3                   (avl_be_3),                   //                   .byteenable
		.avl_read_req_3             (avl_read_req_3),             //                   .read
		.avl_write_req_3            (avl_write_req_3),            //                   .write
		.avl_size_3                 (avl_size_3),                 //                   .burstcount
		.mp_cmd_clk_0_clk           (mp_cmd_clk_0_clk),           //       mp_cmd_clk_0.clk
		.mp_cmd_reset_n_0_reset_n   (mp_cmd_reset_n_0_reset_n),   //   mp_cmd_reset_n_0.reset_n
		.mp_cmd_clk_1_clk           (mp_cmd_clk_1_clk),           //       mp_cmd_clk_1.clk
		.mp_cmd_reset_n_1_reset_n   (mp_cmd_reset_n_1_reset_n),   //   mp_cmd_reset_n_1.reset_n
		.mp_cmd_clk_2_clk           (mp_cmd_clk_2_clk),           //       mp_cmd_clk_2.clk
		.mp_cmd_reset_n_2_reset_n   (mp_cmd_reset_n_2_reset_n),   //   mp_cmd_reset_n_2.reset_n
		.mp_cmd_clk_3_clk           (mp_cmd_clk_3_clk),           //       mp_cmd_clk_3.clk
		.mp_cmd_reset_n_3_reset_n   (mp_cmd_reset_n_3_reset_n),   //   mp_cmd_reset_n_3.reset_n
		.mp_rfifo_clk_0_clk         (mp_rfifo_clk_0_clk),         //     mp_rfifo_clk_0.clk
		.mp_rfifo_reset_n_0_reset_n (mp_rfifo_reset_n_0_reset_n), // mp_rfifo_reset_n_0.reset_n
		.mp_wfifo_clk_0_clk         (mp_wfifo_clk_0_clk),         //     mp_wfifo_clk_0.clk
		.mp_wfifo_reset_n_0_reset_n (mp_wfifo_reset_n_0_reset_n), // mp_wfifo_reset_n_0.reset_n
		.mp_rfifo_clk_1_clk         (mp_rfifo_clk_1_clk),         //     mp_rfifo_clk_1.clk
		.mp_rfifo_reset_n_1_reset_n (mp_rfifo_reset_n_1_reset_n), // mp_rfifo_reset_n_1.reset_n
		.mp_wfifo_clk_1_clk         (mp_wfifo_clk_1_clk),         //     mp_wfifo_clk_1.clk
		.mp_wfifo_reset_n_1_reset_n (mp_wfifo_reset_n_1_reset_n), // mp_wfifo_reset_n_1.reset_n
		.mp_rfifo_clk_2_clk         (mp_rfifo_clk_2_clk),         //     mp_rfifo_clk_2.clk
		.mp_rfifo_reset_n_2_reset_n (mp_rfifo_reset_n_2_reset_n), // mp_rfifo_reset_n_2.reset_n
		.mp_wfifo_clk_2_clk         (mp_wfifo_clk_2_clk),         //     mp_wfifo_clk_2.clk
		.mp_wfifo_reset_n_2_reset_n (mp_wfifo_reset_n_2_reset_n), // mp_wfifo_reset_n_2.reset_n
		.mp_rfifo_clk_3_clk         (mp_rfifo_clk_3_clk),         //     mp_rfifo_clk_3.clk
		.mp_rfifo_reset_n_3_reset_n (mp_rfifo_reset_n_3_reset_n), // mp_rfifo_reset_n_3.reset_n
		.mp_wfifo_clk_3_clk         (mp_wfifo_clk_3_clk),         //     mp_wfifo_clk_3.clk
		.mp_wfifo_reset_n_3_reset_n (mp_wfifo_reset_n_3_reset_n), // mp_wfifo_reset_n_3.reset_n
		.local_init_done            (local_init_done),            //             status.local_init_done
		.local_cal_success          (local_cal_success),          //                   .local_cal_success
		.local_cal_fail             (local_cal_fail),             //                   .local_cal_fail
		.oct_rzqin                  (oct_rzqin),                  //                oct.rzqin
		.pll_mem_clk                (pll_mem_clk),                //        pll_sharing.pll_mem_clk
		.pll_write_clk              (pll_write_clk),              //                   .pll_write_clk
		.pll_locked                 (pll_locked),                 //                   .pll_locked
		.pll_write_clk_pre_phy_clk  (pll_write_clk_pre_phy_clk),  //                   .pll_write_clk_pre_phy_clk
		.pll_addr_cmd_clk           (pll_addr_cmd_clk),           //                   .pll_addr_cmd_clk
		.pll_avl_clk                (pll_avl_clk),                //                   .pll_avl_clk
		.pll_config_clk             (pll_config_clk),             //                   .pll_config_clk
		.pll_mem_phy_clk            (pll_mem_phy_clk),            //                   .pll_mem_phy_clk
		.afi_phy_clk                (afi_phy_clk),                //                   .afi_phy_clk
		.pll_avl_phy_clk            (pll_avl_phy_clk),            //                   .pll_avl_phy_clk
		.seq_debug_addr             (seq_debug_addr),             //          seq_debug.address
		.seq_debug_read_req         (seq_debug_read_req),         //                   .read
		.seq_debug_rdata            (seq_debug_rdata),            //                   .readdata
		.seq_debug_write_req        (seq_debug_write_req),        //                   .write
		.seq_debug_wdata            (seq_debug_wdata),            //                   .writedata
		.seq_debug_waitrequest      (seq_debug_waitrequest),      //                   .waitrequest
		.seq_debug_be               (seq_debug_be),               //                   .byteenable
		.seq_debug_rdata_valid      (seq_debug_rdata_valid)       //                   .readdatavalid
	);

endmodule
