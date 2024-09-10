
`include "defines.vh"
//---------------------------------------------------------------------------
// DUT 
//---------------------------------------------------------------------------
module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//q_state_input SRAM interface
  output wire                                               q_state_input_sram_write_enable  ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_write_address ,
  output wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_write_data    ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_read_address  , 
  input  wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_read_data     ,

//---------------------------------------------------------------------------
//q_state_output SRAM interface
  output wire                                                q_state_output_sram_write_enable  ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address ,
  output wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data    ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address  , 
  input  wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_read_data     ,

//---------------------------------------------------------------------------
//scratchpad SRAM interface                                                       
  output wire                                                scratchpad_sram_write_enable        ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_write_address       ,
  output wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_write_data          ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address        , 
  input  wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_read_data           ,

//---------------------------------------------------------------------------
//q_gates SRAM interface                                                       
  output wire                                                q_gates_sram_write_enable           ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_write_address          ,
  output wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_write_data             ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_read_address           ,  
  input  wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_read_data
);

  //q_state input SRAM interface
  reg                                               q_state_input_sram_write_enable_r;
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_write_address_r;
  reg [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_write_data_r;
  reg [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_read_address_r; 

  //q_state output SRAM interface
  reg                                                q_state_output_sram_write_enable_r;
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address_r;
  reg [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data_r;
  reg [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address_r; 

  //scratch SRAM interface
  reg                                                scratchpad_sram_write_enable_r;
  reg [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_write_address_r;
  reg [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_write_data_r;
  reg [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address_r; 

  //q_gate SRAM interface
  reg                                                q_gates_sram_write_enable_r;
  reg [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_write_address_r;
  reg [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_write_data_r;
  reg [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_read_address_r;         

  localparam inst_sig_width = 52;
  localparam inst_exp_width = 11;
  localparam inst_ieee_compliance = 0;

  reg  [inst_sig_width+inst_exp_width : 0] inst_a;
  reg  [inst_sig_width+inst_exp_width : 0] inst_b;
  reg  [inst_sig_width+inst_exp_width : 0] inst_c;
  reg  [2 : 0] inst_rnd;
  wire [inst_sig_width+inst_exp_width : 0] z_inst;
  wire [7 : 0] status_inst;

  // This is test stub for passing input/outputs to a DP_fp_mac, there many
  // more DW macros that you can choose to use
  DW_fp_mac_inst FP_MAC1 ( 
    inst_a,
    inst_b,
    inst_c,
    inst_rnd,
    z_inst,
    status_inst
  );

  //464 always 2 qbits
  localparam inQ = 2'b10;
  localparam arraySize = 5'b10000;

  //FSM states
  localparam S0 = 3'b000;
  localparam S1 = 3'b001;
  localparam S2 = 3'b010;
  localparam S3 = 3'b011;
  localparam S4 = 3'b100;
  localparam S5 = 3'b101;
  localparam S6 = 3'b110;
  localparam S7 = 3'b111; 

  //FSM Variables
  reg [2:0] curState; 

  //Local Variables
  reg [7:0] numMatrix, curMatrix;
  reg [1:0] curRowA, curRowB;
  reg [255:0] inMatrix, outMatrix;
  reg compute_complete;

  //Next State Logic
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : curState <= !dut_valid ? S0 : S1;                                            //S0 - Wait Idle
        S1 : curState <= S2;                                                              //S1 - Read and Advance q_input read  
        S2 : curState <= q_state_input_sram_read_address_r < 5 ? S1 : S3;                 //S2 - Wait cycle 
        S3 : curState <= S4;                                                              //S3 - Read q_gate
        S4 : curState <= S5;                                                              //S4 - Put in inst_a, b, c - Advance q_gate
        S5 : curState <= (q_gates_sram_read_address_r == (numMatrix << 4)) ? S6 : S3;     //S5 - Save from MAC - check for end of matrix
        S6 : curState <= S7;                                                              //S6 - Write Data to q_out
        S7 : curState <= (q_state_output_sram_write_address_r < 5) ? S6 : S0;             //S7 - Advance q_out address or done
        default : curState <= S0;
      endcase
    end else curState <= S0;
  end

  //Compute Complete ()
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : compute_complete <= dut_valid ? 1'b0 : 1'b1;
        default compute_complete <= compute_complete;
      endcase
    end else compute_complete <= 1'b0;
  end

  assign dut_ready = compute_complete;

  //numMatrix (M)
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : numMatrix <= dut_valid ? q_state_input_sram_read_data[7:0] : 8'b0;
        default : numMatrix <= numMatrix;
      endcase
    end else numMatrix <= 8'b0;
  end

  //curMatrix
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : curMatrix <= 8'b1;
        S4 : curMatrix <= (q_gates_sram_read_address_r == (curMatrix << 4)) ? (curMatrix + 1) : curMatrix;
        default : curMatrix <= curMatrix;
      endcase
    end else curMatrix <= 8'b1;
  end

  //Q_gate read address
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : q_gates_sram_read_address_r <= 32'b0;
        S4 : q_gates_sram_read_address_r <= q_gates_sram_read_address_r + 1;
        default : q_gates_sram_read_address_r <= q_gates_sram_read_address_r;
      endcase
    end else q_gates_sram_read_address_r <= 32'b0;
  end

  assign q_gates_sram_read_address = q_gates_sram_read_address_r;

  //Q_in read address
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : q_state_input_sram_read_address_r <= dut_ready ? 32'b0 : (q_state_input_sram_read_address_r + 1);
        S1 : q_state_input_sram_read_address_r <= q_state_input_sram_read_address_r + 1;
        default : q_state_input_sram_read_address_r <= q_state_input_sram_read_address_r;
      endcase
    end else q_state_input_sram_read_address_r <= 32'b0;
  end

  assign q_state_input_sram_read_address = q_state_input_sram_read_address_r;

  //inMatrix
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : inMatrix <= 256'b0;
        S1 : case (q_state_input_sram_read_address_r)
          12'b001 : inMatrix[63:0]    <= q_state_input_sram_read_data[127:64];
          12'b010 : inMatrix[127:64]  <= q_state_input_sram_read_data[127:64];
          12'b011 : inMatrix[191:128] <= q_state_input_sram_read_data[127:64];
          12'b100 : inMatrix[255:192] <= q_state_input_sram_read_data[127:64];
          default : inMatrix <= 256'b0;
        endcase
        S5 : inMatrix <= (q_gates_sram_read_address_r == (curMatrix << 4)) ? outMatrix : inMatrix;
        default : inMatrix <= inMatrix;
      endcase
    end else inMatrix <= 256'b0;
  end

  //outMatrix
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : outMatrix <= 256'b0;
        S4 : case (curRowA)
          2'b00 : outMatrix[63:0]     <= z_inst;
          2'b01 : outMatrix[127:64]   <= z_inst;
          2'b10 : outMatrix[191:128]  <= z_inst;
          2'b11 : outMatrix[255:192]  <= z_inst;
        endcase
        default : outMatrix <= outMatrix;
      endcase
    end else outMatrix <= 256'b0;
  end

  //curRowA (selects outMatrix slot)
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : curRowA <= 2'b00;
        S5 : curRowA <= curRowB ? curRowA : curRowA + 1;
        default : curRowA <= curRowA;
      endcase
    end else curRowA <= 2'b00;
  end

  //curRowB - cycle inMatrix
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : curRowB <= 2'b0;
        S3 : curRowB <= curRowB + 1;
        default : curRowB <= curRowB;
      endcase
    end else curRowB <= 2'b0;
  end

  //Q_out write address
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : q_state_output_sram_write_address_r <= 32'b0;
        S7 : q_state_output_sram_write_address_r <= q_state_output_sram_write_address_r + 1;
        default : q_state_output_sram_write_address_r <= q_state_output_sram_write_address_r;
      endcase
    end else q_state_output_sram_write_address_r <= 32'b0;
  end

  assign q_state_output_sram_write_address = q_state_output_sram_write_address_r;

  //Q_out write data
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : q_state_output_sram_write_data_r <= 128'b0;
        S6 : begin
          case (q_state_output_sram_write_address_r)
            64'd0 : q_state_output_sram_write_data_r[127:64] <= outMatrix[63:0];
            64'd1 : q_state_output_sram_write_data_r[127:64] <= outMatrix[127:64];
            64'd2 : q_state_output_sram_write_data_r[127:64] <= outMatrix[191:128];
            64'd3 : q_state_output_sram_write_data_r[127:64] <= outMatrix[255:192];
            default : q_state_output_sram_write_data_r <= 128'b0;
          endcase
        end
        default : q_state_output_sram_write_data_r <= q_state_output_sram_write_data_r;
      endcase
    end else q_state_output_sram_write_data_r <= 128'b0;
  end

  assign q_state_output_sram_write_data = q_state_output_sram_write_data_r;

  //Q_out write enable
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : q_state_output_sram_write_enable_r <= 1'b0;
        S6 : q_state_output_sram_write_enable_r <= 1'b1;
        S7 : q_state_output_sram_write_enable_r <= 1'b0;
        default : q_state_output_sram_write_enable_r <= q_state_output_sram_write_enable_r;
      endcase
    end else q_state_output_sram_write_enable_r <= 1'b0;
  end

  assign q_state_output_sram_write_enable = q_state_output_sram_write_enable_r;

  //inst_a
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : inst_a <= 64'b0;
        S3 : inst_a <= q_gates_sram_read_data[127:64];
        S5 : inst_a <= curRowB ? inst_a : 64'b0;
        default : inst_a <= inst_a;
      endcase
    end else inst_a <= 64'b0;
  end

  //inst_b
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : inst_b <= 64'b0;
        S3 : begin
          case (curRowB)
                  2'b00 : inst_b <= inMatrix[63:0];
                  2'b01 : inst_b <= inMatrix[127:64];
                  2'b10 : inst_b <= inMatrix[191:128];
                  2'b11 : inst_b <= inMatrix[255:192];
          endcase
        end
        S5 : inst_b <= curRowB ? inst_b : 64'b0;
        default : inst_b <= inst_b;
      endcase
    end else inst_b <= 64'b0;
  end

  //inst_c
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : inst_c <= 64'b0;
        S3 : inst_c <= z_inst;
        S5 : inst_c <= curRowB ? inst_c : 64'b0;
        default : inst_c <= inst_c;
      endcase
    end else inst_c <= 64'b0;
  end

  //inst_rnd always 0
  always @ (posedge clk) inst_rnd <= 3'b000;

  //Unused SRAM Regs
  always @ (posedge clk) begin
    if (reset_n) begin
      case (curState)
        S0 : begin
          q_state_input_sram_write_address_r <= 32'b0;
          q_state_input_sram_write_data_r <= 64'b0;
          q_state_input_sram_write_enable_r <= 1'b0;
          q_state_output_sram_read_address_r <= 32'b0;
          q_gates_sram_write_address_r <= 32'b0;
          q_gates_sram_write_data_r <= 64'b0;
          q_gates_sram_write_enable_r <= 1'b0;
          scratchpad_sram_read_address_r <= 32'b0;
          scratchpad_sram_write_address_r <= 32'b0;
          scratchpad_sram_write_data_r <= 64'b0;
          scratchpad_sram_write_enable_r <= 1'b0;
        end
        default : begin
          q_state_input_sram_write_address_r <= q_state_input_sram_write_address_r;
          q_state_input_sram_write_data_r <= q_state_input_sram_write_data_r;
          q_state_input_sram_write_enable_r <= q_state_input_sram_write_enable_r;
          q_state_output_sram_read_address_r <= q_state_output_sram_read_address_r;
          q_gates_sram_write_address_r <= q_gates_sram_write_address_r;
          q_gates_sram_write_data_r <= q_gates_sram_write_data_r;
          q_gates_sram_write_enable_r <= q_gates_sram_write_enable_r;
          scratchpad_sram_read_address_r <= scratchpad_sram_read_address_r;
          scratchpad_sram_write_address_r <= scratchpad_sram_write_address_r;
          scratchpad_sram_write_data_r <= scratchpad_sram_write_data_r;
          scratchpad_sram_write_enable_r <= scratchpad_sram_write_enable_r;
        end
      endcase
    end else begin
      q_state_input_sram_write_address_r <= 32'b0;
      q_state_input_sram_write_data_r <= 64'b0;
      q_state_input_sram_write_enable_r <= 1'b0;
      q_state_output_sram_read_address_r <= 32'b0;
      q_gates_sram_write_address_r <= 32'b0;
      q_gates_sram_write_data_r <= 64'b0;
      q_gates_sram_write_enable_r <= 1'b0;
      scratchpad_sram_read_address_r <= 32'b0;
      scratchpad_sram_write_address_r <= 32'b0;
      scratchpad_sram_write_data_r <= 64'b0;
      scratchpad_sram_write_enable_r <= 1'b0;
    end
  end

  assign q_state_input_sram_write_address = q_state_input_sram_write_address_r;
  assign q_state_input_sram_write_data = q_state_input_sram_write_data_r;
  assign q_state_input_sram_write_enable = q_state_input_sram_write_enable_r;
  assign q_state_output_sram_read_address = q_state_output_sram_read_address_r;
  assign q_gates_sram_write_address = q_gates_sram_write_address_r;
  assign q_gates_sram_write_data = q_gates_sram_write_data_r;
  assign q_gates_sram_write_enable = q_gates_sram_write_enable_r;
  assign scratchpad_sram_read_address = scratchpad_sram_read_address_r;
  assign scratchpad_sram_write_address = scratchpad_sram_write_address_r;
  assign scratchpad_sram_write_data = scratchpad_sram_write_data_r;
  assign scratchpad_sram_write_enable = scratchpad_sram_write_enable_r;

endmodule

module DW_fp_mac_inst #(
  parameter inst_sig_width = 52,
  parameter inst_exp_width = 11,
  parameter inst_ieee_compliance = 1 // These need to be fixed to decrease error
) ( 
  input wire [inst_sig_width+inst_exp_width : 0] inst_a,
  input wire [inst_sig_width+inst_exp_width : 0] inst_b,
  input wire [inst_sig_width+inst_exp_width : 0] inst_c,
  input wire [2 : 0] inst_rnd,
  output wire [inst_sig_width+inst_exp_width : 0] z_inst,
  output wire [7 : 0] status_inst
);

  // Instance of DW_fp_mac
  DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
    .a(inst_a),
    .b(inst_b),
    .c(inst_c),
    .rnd(inst_rnd),
    .z(z_inst),
    .status(status_inst) 
  );

endmodule