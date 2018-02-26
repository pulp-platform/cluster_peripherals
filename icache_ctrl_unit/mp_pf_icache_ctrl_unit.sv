// Copyright 2014-2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// ============================================================================= //
// Company:        Multitherman Laboratory @ DEIS - University of Bologna        //
//                    Viale Risorgimento 2 40136                                 //
//                    Bologna - fax 0512093785 -                                 //
//                                                                               //
// Engineer:       Igor Loi - igor.loi@unibo.it                                  //
//                                                                               //
// Additional contributions by:                                                  //
//                                                                               //
// Create Date:    18/08/2014                                                    // 
// Design Name:    icache_ctrl_unit                                              // 
// Module Name:    icache_ctrl_unit                                              //
// Project Name:   PULP                                                          //
// Language:       SystemVerilog                                                 //
//                                                                               //
// Description:    ICACHE control Unit, used to enable/disable icache banks      //
//                 flush operations, and to debug the status og cache banks      //
//                                                                               //
// Revision:                                                                     //
// Revision v0.1 - File Created                                                  //
//                                                                               //
// ============================================================================= //

`include "pulp_soc_defines.sv"
    `define ENABLE_ICACHE     6'b00_0000
    `define FLUSH_ICACHE      6'b00_0001
    `define SEL_FLUSH_ICACHE  6'b00_0011

    `define PF_SIZE           6'b00_1000
    `define PF_ADDR           6'b00_1001

`ifdef FEATURE_ICACHE_STAT  //TO BE TESTED DEEPLY
    `define CLEAR_CNTS        6'b00_0100
    `define ENABLE_CNTS       6'b00_0101

    `define READ_ICACHE_HIT_CORES     6'b01_0000 // R Only
    `define READ_ICACHE_TRANS_CORES   6'b01_0001 // R Only
    `define READ_ICACHE_MISS_CORES    6'b01_0001 // R Only
`endif


//-----------------------------------//


module mp_pf_icache_ctrl_unit
#(
    parameter  NB_CACHE_BANKS = 4,
    parameter  NB_CORES       = 4,
    parameter  ID_WIDTH       = 5
)
(
    input logic                                 clk_i,
    input logic                                 rst_ni,

    XBAR_PERIPH_BUS.Slave                       speriph_slave,
    MP_PF_ICACHE_CTRL_UNIT_BUS.Master           IC_ctrl_unit_master_if,

    output logic                                pf_event_o
);

    int unsigned                             i,j,k,x,y;

`ifdef FEATURE_ICACHE_STAT
    localparam                          NUM_REGS       = 16;
`else
    localparam                          NUM_REGS       = 16;
`endif



    logic                                icache_bypass_req_o;
    logic [NB_CORES:0]                   icache_bypass_ack_i;

    logic                                icache_flush_req_o;
    logic                                icache_flush_ack_i;

    logic                                icache_sel_flush_req_o;
    logic [31:0]                         icache_sel_flush_addr_o;
    logic                                icache_sel_flush_ack_i;

    logic [31:0]                         pf_addr_o;
    logic [7:0]                          pf_size_o;
    logic                                pf_req_o;
    logic                                pf_ack_i;
    logic                                pf_done_i;


    logic [31:0]                ICACHE_CTRL_REGS[NUM_REGS];

    // State of the main FSM
`ifdef FEATURE_ICACHE_STAT
    enum logic [2:0] { IDLE, ENABLE_ICACHE,  DISABLE_ICACHE, FLUSH_ICACHE_CHECK, SEL_FLUSH_ICACHE, CLEAR_STAT_REGS, ENABLE_STAT_REGS, TRIGGER_PF } CS, NS;
`else
    enum logic [2:0] { IDLE, ENABLE_ICACHE,  DISABLE_ICACHE, FLUSH_ICACHE_CHECK, SEL_FLUSH_ICACHE, TRIGGER_PF } CS, NS;
`endif

    // Exploded Interface --> PERIPHERAL INTERFACE
    logic                req;
    logic [31:0]         addr;
    logic                wen;
    logic [31:0]         wdata;
    logic [3:0]          be;
    logic                gnt;
    logic [ID_WIDTH-1:0] id;
    logic                r_valid;
    logic                r_opc;
    logic [ID_WIDTH-1:0] r_id;
    logic [31:0]         r_rdata;


    // Internal FSM signals --> responses
    logic                                       r_valid_int;
    logic [31:0]                                r_rdata_int;

`ifdef FEATURE_ICACHE_STAT
    logic [NB_CORES-1:0] [31:0]                 hit_count;
    logic [NB_CORES-1:0] [31:0]                 trans_count;
    logic [NB_CORES-1:0] [31:0]                 miss_count;
    logic [NB_CORES-1:0]                        clear_regs;
    logic [NB_CORES-1:0]                        enable_regs;

    logic [31:0]                                global_hit_count;
    logic [31:0]                                global_trans_count;
    logic [31:0]                                global_miss_count;

    logic [7:0][31:0]                           bank_hit_count;
    logic [7:0][31:0]                           bank_trans_count;
    logic [7:0][31:0]                           bank_miss_count;
`endif

    logic                                       is_write;
    logic                                       deliver_response;
    logic                                       clear_flush_reg;


    // Interface binding
    assign speriph_slave.gnt                    = gnt;
    assign req                                  = speriph_slave.req;
    assign addr                                 = speriph_slave.add;
    assign wen                                  = speriph_slave.wen;
    assign wdata                                = speriph_slave.wdata;
    assign be                                   = speriph_slave.be;
    assign id                                   = speriph_slave.id;
    assign speriph_slave.r_valid                = r_valid;
    assign speriph_slave.r_opc                  = r_opc;
    assign speriph_slave.r_id                   = r_id;
    assign speriph_slave.r_rdata                = r_rdata;




   genvar index;

    assign IC_ctrl_unit_master_if.pf_addr  = pf_addr_o;
    assign IC_ctrl_unit_master_if.pf_size  = pf_size_o;
    assign IC_ctrl_unit_master_if.pf_req   = pf_req_o;
    assign pf_ack_i  = IC_ctrl_unit_master_if.pf_ack;
    assign pf_done_i = IC_ctrl_unit_master_if.pf_done;

    assign IC_ctrl_unit_master_if.bypass_req       = icache_bypass_req_o;
    assign icache_bypass_ack_i                     = IC_ctrl_unit_master_if.bypass_ack;
    assign IC_ctrl_unit_master_if.flush_req        = icache_flush_req_o;
    assign icache_flush_ack_i                      = IC_ctrl_unit_master_if.flush_ack;

    assign IC_ctrl_unit_master_if.sel_flush_req    = icache_sel_flush_req_o;
    assign IC_ctrl_unit_master_if.sel_flush_addr   = icache_sel_flush_addr_o;
    assign icache_sel_flush_ack_i                  = IC_ctrl_unit_master_if.sel_flush_ack;


`ifdef FEATURE_ICACHE_STAT
    assign IC_ctrl_unit_master_if.ctrl_clear_regs  = clear_regs;
    assign IC_ctrl_unit_master_if.ctrl_enable_regs = enable_regs;

    assign global_hit_count    = IC_ctrl_unit_master_if.global_hit_count;
    assign global_trans_count  = IC_ctrl_unit_master_if.global_trans_count; 
    assign global_miss_count   = IC_ctrl_unit_master_if.global_miss_count; 
  
  generate

    assign bank_hit_count[NB_CORES-1:0]    = IC_ctrl_unit_master_if.bank_hit_count;
    assign bank_trans_count[NB_CORES-1:0]  = IC_ctrl_unit_master_if.bank_trans_count; 
    assign bank_miss_count[NB_CORES-1:0]   = IC_ctrl_unit_master_if.bank_miss_count;

  endgenerate
`endif



 
   always_comb
   begin : REGISTER_BIND_OUT
      icache_bypass_req_o     =  ~ICACHE_CTRL_REGS[`ENABLE_ICACHE];
      icache_flush_req_o      =   ICACHE_CTRL_REGS[`FLUSH_ICACHE];
      icache_sel_flush_addr_o =   ICACHE_CTRL_REGS[`SEL_FLUSH_ICACHE];

      pf_addr_o               =   ICACHE_CTRL_REGS[`PF_ADDR][31:0];
      pf_size_o               =   ICACHE_CTRL_REGS[`PF_SIZE][7:0];

`ifdef FEATURE_ICACHE_STAT
      enable_regs =   ICACHE_CTRL_REGS[`ENABLE_CNTS][NB_CACHE_BANKS-1:0];
`endif      
   end



   always_ff @(posedge clk_i, negedge rst_ni)
   begin : SEQ_PROC
      if(rst_ni == 1'b0)
      begin
              CS                  <= IDLE;
              r_id                <= '0;
              r_valid             <= 1'b0;
              r_rdata             <= '0;
              r_opc               <= '0;

              for(i=0;i<NUM_REGS;i++)
              begin
                ICACHE_CTRL_REGS[i] <= '0;
              end
      end
      else
      begin

        CS                  <= NS;

        if(is_write)
        begin
            case(addr[7:2])
                `ENABLE_ICACHE: // ENABLE-DISABLE
                begin
                      ICACHE_CTRL_REGS[`ENABLE_ICACHE] <= {31'h0000_0000, wdata[0]};
                end

                `FLUSH_ICACHE: // FLUSH
                begin
                  ICACHE_CTRL_REGS[`FLUSH_ICACHE] <= {31'h0000_0000, wdata[0]};
                end

                `SEL_FLUSH_ICACHE: // Sel FLUSH
                begin
                  ICACHE_CTRL_REGS[`SEL_FLUSH_ICACHE] <= wdata;
                end

                `PF_SIZE:
                begin
                  ICACHE_CTRL_REGS[`PF_SIZE] <= {28'h00_0000,wdata[7:0]};
                end

                `PF_ADDR:
                begin
                  ICACHE_CTRL_REGS[`PF_ADDR] <= wdata[31:0];
                end

            `ifdef FEATURE_ICACHE_STAT
                `CLEAR_CNTS: // CLEAR
                begin
                  ICACHE_CTRL_REGS[`CLEAR_CNTS] <= wdata;
                end

                `ENABLE_CNTS: // ENABLE-DISABLE STAT REGS
                begin
                  ICACHE_CTRL_REGS[`ENABLE_CNTS] <= wdata;
                end
            `endif
            endcase
        end
        else // Not Write
        begin
            if(clear_flush_reg)
               ICACHE_CTRL_REGS[`FLUSH_ICACHE] <= 32'h0000_0000;
        end






        // sample the ID
        if(req & gnt)
        begin
          r_id    <= id;
        end


        //Handle register read
        if(deliver_response == 1'b1)
        begin
          r_valid <= 1'b1;

          case(addr[7:2])
          `ENABLE_ICACHE:  begin r_rdata <= ICACHE_CTRL_REGS[`ENABLE_ICACHE]; end
          `FLUSH_ICACHE:   begin r_rdata <= ICACHE_CTRL_REGS[`FLUSH_ICACHE];  end
          `PF_SIZE:        begin r_rdata <= ICACHE_CTRL_REGS[`PF_SIZE];       end
          `PF_ADDR:        begin r_rdata <= ICACHE_CTRL_REGS[`PF_ADDR];       end

    `ifdef FEATURE_ICACHE_STAT
          // Clear and start
          `CLEAR_CNTS:  begin r_rdata <= ICACHE_CTRL_REGS[`CLEAR_CNTS];  end
          `ENABLE_CNTS:  begin r_rdata <= ICACHE_CTRL_REGS[`ENABLE_CNTS]; end

          16:  begin r_rdata <= global_hit_count   ;  end
          17:  begin r_rdata <= global_trans_count ;  end
          18:  begin r_rdata <= global_miss_count  ;  end
          19:  begin r_rdata <= 32'hFFFF_FFFF;     ;  end
    `endif
          default : begin r_rdata <= 32'hDEAD_CACE; end
          endcase
          r_opc   <= 1'b0;
        end
        else //nothing to Do
        begin
                  r_valid <= 1'b0;
                  r_opc   <= 1'b0;
        end

      end
   end




   always_comb
   begin
        is_write               = 1'b0;
        deliver_response       = 1'b0;
        gnt                    = 1'b0;
        icache_sel_flush_req_o = 1'b0;
        clear_flush_reg        = 1'b0;

        pf_req_o               = 1'b0;


`ifdef FEATURE_ICACHE_STAT
        clear_regs             = '0;
`endif

        case(CS)

          IDLE:
          begin
              gnt = 1'b1;

              if(req)
              begin
                if(wen == 1'b1) // read
                begin
                      NS               = IDLE;
                      deliver_response = 1'b1;
                end
                else // Write registers
                begin

                      is_write = 1'b1;

                      case(addr[7:2])
                        `ENABLE_ICACHE: // Enable - Disable register
                        begin
                            if(wdata == 0)
                              NS = DISABLE_ICACHE;
                            else
                              NS = ENABLE_ICACHE;
                        end //~2'b00

                        `FLUSH_ICACHE:
                        begin
                          NS = FLUSH_ICACHE_CHECK;
                        end

                        `SEL_FLUSH_ICACHE:
                        begin
                          NS = SEL_FLUSH_ICACHE;
                        end

                        `PF_SIZE:
                        begin
                          deliver_response = 1'b1;
                          NS = IDLE;
                        end

                        `PF_ADDR:
                        begin
                          NS = TRIGGER_PF;
                        end



                    `ifdef FEATURE_ICACHE_STAT
                        `CLEAR_CNTS: // CLEAR
                        begin
                          NS = CLEAR_STAT_REGS;
                        end

                        `ENABLE_CNTS: // START
                        begin
                          NS = ENABLE_STAT_REGS;
                        end
                    `endif


                        default:
                        begin
                          NS               = IDLE;
                        end
                      endcase

                end

              end
              else // no request
              begin
                  NS = IDLE;
              end

          end //~IDLE

`ifdef FEATURE_ICACHE_STAT
          CLEAR_STAT_REGS:
          begin
             for(x=0; x<NB_CACHE_BANKS; x++)
             begin
                clear_regs[x]  =   ICACHE_CTRL_REGS[`CLEAR_CNTS][x];
             end

             deliver_response = 1'b1;
             NS = IDLE;
          end //~ CLEAR_STAT_REGS


          ENABLE_STAT_REGS:
          begin

             deliver_response = 1'b1;
             NS = IDLE;
          end //~ENABLE_STAT_REGS
`endif


          ENABLE_ICACHE: 
          begin
            gnt = 1'b0;
            if(|icache_bypass_ack_i == 1'b0) //11111 --> all bypassed; 00000 --> all enabled
            begin
              NS = IDLE;
              deliver_response = 1'b1;
            end
            else
            begin
              NS = ENABLE_ICACHE;
            end
          end //~ENABLE_ICACHE



          DISABLE_ICACHE: 
          begin
            gnt = 1'b0;

            if(&icache_bypass_ack_i == 1'b1) //11111 --> all bypassed; 00000 --> all enabled
            begin
              NS = IDLE;
              deliver_response = 1'b1;
            end
            else
            begin
              NS = DISABLE_ICACHE;
            end
          end //~DIABLE_ICACHE




          FLUSH_ICACHE_CHECK:
          begin
              gnt = 1'b0;

              if(icache_flush_ack_i)
              begin
                NS = IDLE;
                deliver_response = 1'b1;
                 clear_flush_reg  = 1'b1;
              end
              else
              begin
                NS = FLUSH_ICACHE_CHECK;
              end
          end


          SEL_FLUSH_ICACHE:
          begin

              if(icache_sel_flush_ack_i)
              begin
                gnt = 1'b1;
                NS  = IDLE;
               
                deliver_response = 1'b1;
              end
              else
              begin
                NS = SEL_FLUSH_ICACHE;
              end
          end

          TRIGGER_PF:
          begin
              pf_req_o = 1'b1;

              if(pf_ack_i)
              begin
                NS = IDLE;
                deliver_response = 1'b1;
              end
              else
              begin
                NS = TRIGGER_PF;
              end
          end


        default :
        begin
                NS = IDLE;
        end
        endcase
   end

   assign pf_event_o = pf_done_i;

endmodule
