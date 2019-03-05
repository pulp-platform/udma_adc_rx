// Copyright 2016 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

////////////////////////////////////////////////////////////////////////////////
// Engineer:       Florian Glaser - glaserf@ethz.ch                           //
//                                                                            //
// Additional contributions by:                                               //
//                                                                            //
//                                                                            //
// Design Name:    ADC Register Programming Interface                         //
// Project Name:   uDMA ADC rx channel                                        //
// Language:       SystemVerilog                                              //
//                                                                            //
// Description:    simple uDMA ADC interface to sample ADC data               //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

// register map
`define REG_RX_SADDR     3'b000 //BASEADDR+0x00
`define REG_RX_SIZE      3'b001 //BASEADDR+0x04
`define REG_RX_CFG       3'b010 //BASEADDR+0x08
`define REG_RX_INTCFG    3'b011 //BASEADDR+0x0C
`define REG_SINGLE_CH    3'b100 //BASEADDR+0x10

module udma_adc_rx_reg_if #(
    parameter L2_AWIDTH_NOAL = 12,
    parameter TRANS_SIZE     = 16,
    parameter ADC_NUM_CHS    = 8
) (
	input  logic 	                  clk_i,
	input  logic   	                  rstn_i,

	input  logic               [31:0] cfg_data_i,
	input  logic                [7:0] cfg_addr_i,
	input  logic                      cfg_valid_i,
	input  logic                      cfg_rwn_i,
    output logic               [31:0] cfg_data_o,
	output logic                      cfg_ready_o,

    output logic [ADC_NUM_CHS-1:0][L2_AWIDTH_NOAL-1:0] cfg_rx_startaddr_o,
    output logic [ADC_NUM_CHS-1:0]    [TRANS_SIZE-1:0] cfg_rx_size_o,
    output logic [ADC_NUM_CHS-1:0]               [1:0] cfg_rx_datasize_o,
    output logic [ADC_NUM_CHS-1:0]                     cfg_rx_continuous_o,
    output logic [ADC_NUM_CHS-1:0]                     cfg_rx_en_o,
    output logic [ADC_NUM_CHS-1:0]                     cfg_rx_clr_o,
    input  logic [ADC_NUM_CHS-1:0]                     cfg_rx_en_i,
    input  logic [ADC_NUM_CHS-1:0]                     cfg_rx_pending_i,
    input  logic [ADC_NUM_CHS-1:0][L2_AWIDTH_NOAL-1:0] cfg_rx_curr_addr_i,
    input  logic [ADC_NUM_CHS-1:0]    [TRANS_SIZE-1:0] cfg_rx_bytes_left_i,

    output logic cfg_single_ch_mode_o
);

    logic [ADC_NUM_CHS-1:0][L2_AWIDTH_NOAL-1:0] r_rx_startaddr;
    logic [ADC_NUM_CHS-1:0]    [TRANS_SIZE-1:0] r_rx_size;
    logic [ADC_NUM_CHS-1:0]                     r_rx_continuous;
    logic [ADC_NUM_CHS-1:0]                     r_rx_en;
    logic [ADC_NUM_CHS-1:0]                     r_rx_clr;

    logic        r_single_ch_mode;


    logic  [2:0] s_reg_sel_w, s_reg_sel_r;
    logic  [4:0] s_ch_sel_w, s_ch_sel_r;

    assign s_reg_sel_w = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i[2:0] : 'h0;
    assign s_reg_sel_r = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i[2:0] : 'h0;

    assign s_ch_sel_w  = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i[7:3] : 'h0;
    assign s_ch_sel_r  = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i[7:3] : 'h0;

    assign cfg_rx_startaddr_o   = r_rx_startaddr;
    assign cfg_rx_size_o        = r_rx_size;
    assign cfg_rx_datasize_o    = {ADC_NUM_CHS{2'b10}};
    assign cfg_rx_continuous_o  = r_rx_continuous;
    assign cfg_rx_en_o          = r_rx_en;
    assign cfg_rx_clr_o         = r_rx_clr;
    assign cfg_single_ch_mode_o = r_single_ch_mode;

    always_ff @(posedge clk_i, negedge rstn_i) begin
        if(~rstn_i) begin
            // SPI REGS
            r_rx_startaddr  <=  'h0;
            r_rx_size       <=  'h0;
            r_rx_continuous <=  'h0;
            r_rx_en          =  'h0;
            r_rx_clr         =  'h0;
            r_single_ch_mode = 1'b0;
        end 
        else begin
            r_rx_en          =  'h0;
            r_rx_clr         =  'h0;

            if (cfg_valid_i & ~cfg_rwn_i) begin
               case (s_reg_sel_w)
                    `REG_RX_SADDR:
                        r_rx_startaddr[s_ch_sel_w]   <= cfg_data_i[L2_AWIDTH_NOAL-1:0];
                    `REG_RX_SIZE:
                        r_rx_size[s_ch_sel_w]        <= cfg_data_i[TRANS_SIZE-1:0];
                    `REG_RX_CFG:
                    begin
                        r_rx_clr[s_ch_sel_w]          = cfg_data_i[5];
                        r_rx_en[s_ch_sel_w]           = cfg_data_i[4];
                        r_rx_continuous[s_ch_sel_w]  <= cfg_data_i[0];
                    end
                    `REG_SINGLE_CH:
                        r_single_ch_mode             <= cfg_data_i[0];
               endcase
            end
        end
    end //always

    always_comb begin
        cfg_data_o = '0;

        case (s_reg_sel_r)
            `REG_RX_SADDR:
                cfg_data_o = cfg_rx_curr_addr_i[s_ch_sel_r];
            `REG_RX_SIZE:
                cfg_data_o[TRANS_SIZE-1:0] = cfg_rx_bytes_left_i[s_ch_sel_r];
            `REG_RX_CFG:
                cfg_data_o = {26'h0,cfg_rx_pending_i[s_ch_sel_r],cfg_rx_en_i[s_ch_sel_r],1'b0,2'b10,r_rx_continuous[s_ch_sel_r]};
            `REG_SINGLE_CH:
                cfg_data_o = {31'h0,r_single_ch_mode};
            default:
                cfg_data_o = '0;
        endcase
    end

    assign cfg_ready_o  = 1'b1;

endmodule 
