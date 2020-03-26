// Copyright 2016 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the “License”); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

module udma_adc_rx_top #(
  parameter L2_AWIDTH_NOAL  = 12,
  parameter UDMA_TRANS_SIZE = 16,
  parameter TRANS_SIZE      = 16,
  parameter ADC_DATA_WIDTH  = 32,
  parameter ADC_NUM_CHS     = 8,
  parameter CH_ID_LSB       = 28,
  parameter CH_ID_WIDTH     = 4  )
(
  input  logic                      sys_clk_i,      // master clock
  input  logic                      periph_clk_i,   // master clock
  input  logic                      rst_ni,         // asynchronous active low reset

  input  logic               [31:0] cfg_data_i,
  input  logic                [7:0] cfg_addr_i,
  input  logic                      cfg_valid_i,
  input  logic                      cfg_rwn_i,
  output logic               [31:0] cfg_data_o,
  output logic                      cfg_ready_o,

  output logic [ADC_NUM_CHS-1:0] [L2_AWIDTH_NOAL-1:0] cfg_rx_startaddr_o,
  output logic [ADC_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] cfg_rx_size_o,
  output logic [ADC_NUM_CHS-1:0]                      cfg_rx_continuous_o,
  output logic [ADC_NUM_CHS-1:0]                      cfg_rx_en_o,
  output logic [ADC_NUM_CHS-1:0]                      cfg_rx_clr_o,
  input  logic [ADC_NUM_CHS-1:0]                      cfg_rx_en_i,
  input  logic [ADC_NUM_CHS-1:0]                      cfg_rx_pending_i,
  input  logic [ADC_NUM_CHS-1:0] [L2_AWIDTH_NOAL-1:0] cfg_rx_curr_addr_i,
  input  logic [ADC_NUM_CHS-1:0][UDMA_TRANS_SIZE-1:0] cfg_rx_bytes_left_i,
              
  output logic [ADC_NUM_CHS-1:0]                [1:0] data_rx_datasize_o,
  output logic [ADC_NUM_CHS-1:0]               [31:0] data_rx_o,
  output logic [ADC_NUM_CHS-1:0]                      data_rx_valid_o,
  input  logic [ADC_NUM_CHS-1:0]                      data_rx_ready_i,

  // ADC signals
  input  logic                      adc_rx_valid_async_i,
  output logic                      adc_rx_valid_sync_o,
  input  logic [ADC_DATA_WIDTH-1:0] adc_rx_data_i,
  output logic [ADC_DATA_WIDTH-1:0] adc_rx_data_reg_o
);


  logic                 [2:0] adc_data_valid_sync;
  logic  [ADC_DATA_WIDTH-1:0] adc_data_sync;
  logic     [ADC_NUM_CHS-1:0] adc_udma_valid_SP, adc_udma_valid_SN;

  logic     adc_vld_edge, adc_vld_edge_del;
  logic     single_ch_mode;


  assign adc_rx_valid_sync_o = adc_vld_edge_del;
  assign adc_rx_data_reg_o   = adc_data_sync;

  assign data_rx_valid_o     = adc_udma_valid_SP;

  always_comb begin
    for (int i=0; i<ADC_NUM_CHS; i++) data_rx_o[i] = {{(32-ADC_DATA_WIDTH){1'b0}}, adc_data_sync};
  end

  // sync & edge detect of adc_data_valid_i
  always_ff @(posedge sys_clk_i, negedge rst_ni) begin
    if ( rst_ni == 1'b0 ) begin
      adc_data_valid_sync    <= '0;
      adc_vld_edge_del       <= '0;
      adc_udma_valid_SP      <= '0;
    end
    else begin
      adc_data_valid_sync[0] <= adc_rx_valid_async_i;
      adc_data_valid_sync[1] <= adc_data_valid_sync[0];
      adc_data_valid_sync[2] <= adc_data_valid_sync[1];
      adc_vld_edge_del       <= adc_vld_edge;
      adc_udma_valid_SP      <= adc_udma_valid_SN;

      if (adc_vld_edge) adc_data_sync <= adc_rx_data_i;

    end
  end


  assign adc_vld_edge = adc_data_valid_sync[1] & ~adc_data_valid_sync[2];

  generate

    if (ADC_NUM_CHS == 1) begin
      always_comb begin
        adc_udma_valid_SN = adc_udma_valid_SP;

        if (adc_vld_edge)
          adc_udma_valid_SN[0] = 1'b1;
        else if (data_rx_ready_i[0])
          adc_udma_valid_SN[0] = 1'b0;
      end
    end
    else begin
      always_comb begin
        adc_udma_valid_SN = adc_udma_valid_SP;

        if (single_ch_mode) begin
          if (adc_vld_edge)
            adc_udma_valid_SN[0] = 1'b1;
          else if (data_rx_ready_i[0])
            adc_udma_valid_SN[0] = 1'b0;
        end
        else begin
          for (int i=0; i<ADC_NUM_CHS; i++) begin
            if ( (adc_vld_edge) && (adc_rx_data_i[CH_ID_LSB+CH_ID_WIDTH-1:CH_ID_LSB] == i) )
              adc_udma_valid_SN[i] = 1'b1;
            else if (data_rx_ready_i[i])
              adc_udma_valid_SN[i] = 1'b0;
          end
        end

      end

    end

  endgenerate


  udma_adc_rx_reg_if #(
    .L2_AWIDTH_NOAL  ( L2_AWIDTH_NOAL  ),
    .UDMA_TRANS_SIZE ( UDMA_TRANS_SIZE ),
    .TRANS_SIZE      ( TRANS_SIZE      ),
    .ADC_NUM_CHS     ( ADC_NUM_CHS     )
  ) udma_adc_rx_reg_if_i
  (
    .clk_i               ( sys_clk_i            ),
    .rstn_i              ( rst_ni               ),

    .cfg_data_i          ( cfg_data_i           ),
    .cfg_addr_i          ( cfg_addr_i           ),
    .cfg_valid_i         ( cfg_valid_i          ),
    .cfg_rwn_i           ( cfg_rwn_i            ),
    .cfg_data_o          ( cfg_data_o           ),
    .cfg_ready_o         ( cfg_ready_o          ),

    .cfg_rx_startaddr_o  ( cfg_rx_startaddr_o   ),
    .cfg_rx_size_o       ( cfg_rx_size_o        ),
    .cfg_rx_datasize_o   ( data_rx_datasize_o   ),
    .cfg_rx_continuous_o ( cfg_rx_continuous_o  ),
    .cfg_rx_en_o         ( cfg_rx_en_o          ),
    .cfg_rx_clr_o        ( cfg_rx_clr_o         ),
    .cfg_rx_en_i         ( cfg_rx_en_i          ),
    .cfg_rx_pending_i    ( cfg_rx_pending_i     ),
    .cfg_rx_curr_addr_i  ( cfg_rx_curr_addr_i   ),
    .cfg_rx_bytes_left_i ( cfg_rx_bytes_left_i  ),

    .cfg_single_ch_mode_o( single_ch_mode       )
  );


endmodule