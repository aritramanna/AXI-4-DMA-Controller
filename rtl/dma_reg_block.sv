`timescale 1ns/1ps

module dma_reg_block #(
    parameter AXI_ADDR_W = 32
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // AXI4-Lite Slave Interface
    input  logic [31:0]             cfg_s_axi_awaddr,
    input  logic                    cfg_s_axi_awvalid,
    output logic                    cfg_s_axi_awready,
    input  logic [31:0]             cfg_s_axi_wdata,
    input  logic [3:0]              cfg_s_axi_wstrb,
    input  logic                    cfg_s_axi_wvalid,
    output logic                    cfg_s_axi_wready,
    output logic [1:0]              cfg_s_axi_bresp,
    output logic                    cfg_s_axi_bvalid,
    input  logic                    cfg_s_axi_bready,
    input  logic [31:0]             cfg_s_axi_araddr,
    input  logic                    cfg_s_axi_arvalid,
    output logic                    cfg_s_axi_arready,
    output logic [31:0]             cfg_s_axi_rdata,
    output logic [1:0]              cfg_s_axi_rresp,
    output logic                    cfg_s_axi_rvalid,
    input  logic                    cfg_s_axi_rready,
    
    // Core Interface
    output logic                    core_start,
    output logic [31:0]             core_src_addr,
    output logic [31:0]             core_dst_addr,
    output logic [31:0]             core_len,
    input  logic                    core_done,    // Pulse
    input  logic                    core_busy,    // Level
    input  logic [3:0]              core_status,  // Valid when done
    
    // Interrupt Output
    output logic                    intr_pend
);

    // Registers
    logic [31:0] reg_ctrl;     // 0x04
    logic [31:0] reg_src_addr; // 0x0C
    logic [31:0] reg_dst_addr; // 0x10
    logic [31:0] reg_len;      // 0x14
    
    // Internal Status State (Sticky)
    logic sts_done;
    logic sts_error;
    logic [3:0] sts_err_code;
    
    // AXI Handshake State
    logic aw_en;
    
    //--------------------------------------------------------------------------
    // Write Channel
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_s_axi_awready <= 1'b0;
            cfg_s_axi_wready  <= 1'b0;
            cfg_s_axi_bvalid  <= 1'b0;
            cfg_s_axi_bresp   <= 2'b00;
            aw_en             <= 1'b1;
            
            reg_ctrl      <= '0;
            reg_src_addr  <= '0;
            reg_dst_addr  <= '0;
            reg_len       <= '0;
            
            sts_done      <= 1'b0;
            sts_error     <= 1'b0;
            sts_err_code  <= 4'h0;
            
            core_start    <= 1'b0;
        end else begin
            // Handshakes
            if (~cfg_s_axi_awready && cfg_s_axi_awvalid && cfg_s_axi_wvalid && aw_en) begin
                cfg_s_axi_awready <= 1'b1;
                cfg_s_axi_wready  <= 1'b1;
                aw_en             <= 1'b0;
            end else begin
                cfg_s_axi_awready <= 1'b0;
                cfg_s_axi_wready  <= 1'b0;
            end
            
            if (cfg_s_axi_bvalid && cfg_s_axi_bready) begin
                cfg_s_axi_bvalid <= 1'b0;
                aw_en            <= 1'b1;
            end else if (cfg_s_axi_awready && cfg_s_axi_wready) begin
                cfg_s_axi_bvalid <= 1'b1;
                // Check if address is valid
                case (cfg_s_axi_awaddr[7:0])
                    8'h04, 8'h08, 8'h0C, 8'h10, 8'h14: cfg_s_axi_bresp <= 2'b00; // OKAY
                    default:                           cfg_s_axi_bresp <= 2'b10; // SLVERR
                endcase
            end
            
            // Core Start Pulse Logic (Self-Clearing)
            core_start <= 1'b0;
            
            // Register Writes
            if (cfg_s_axi_awready && cfg_s_axi_wready) begin
                case (cfg_s_axi_awaddr[7:0])
                    8'h04: begin // Control
                        // Bit 0: Start
                        // BLOCKING LOGIC: Only assert start if no interrupt is pending
                        if (cfg_s_axi_wdata[0] && !intr_pend) begin
                             core_start <= 1'b1; 
                        end
                        // Bit 1: Int Enable
                        reg_ctrl[1] <= cfg_s_axi_wdata[1];
                    end
                    8'h08: begin // Status (W1C)
                        if (cfg_s_axi_wdata[0]) sts_done  <= 1'b0;
                        if (cfg_s_axi_wdata[2]) sts_error <= 1'b0;
                    end
                    8'h0C: reg_src_addr <= cfg_s_axi_wdata;
                    8'h10: reg_dst_addr <= cfg_s_axi_wdata;
                    8'h14: reg_len      <= cfg_s_axi_wdata;
                    default: ;
                endcase
            end
            
            // Core Feedback (Sticky Set)
            if (core_done) begin
                sts_done <= 1'b1;
                sts_err_code <= core_status; // Always capture status (0 or Error)
                if (core_status != 4'h0) begin
                    sts_error    <= 1'b1;
                end
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Read Channel
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_s_axi_arready <= 1'b0;
            cfg_s_axi_rvalid  <= 1'b0;
            cfg_s_axi_rdata   <= '0;
            cfg_s_axi_rresp   <= 2'b00;
        end else begin
            if (~cfg_s_axi_arready && cfg_s_axi_arvalid) begin
                cfg_s_axi_arready <= 1'b1;
            end else begin
                cfg_s_axi_arready <= 1'b0;
            end
            
            if (cfg_s_axi_arready && cfg_s_axi_arvalid && ~cfg_s_axi_rvalid) begin
                cfg_s_axi_rvalid <= 1'b1;
                cfg_s_axi_rresp  <= 2'b00;
                
                case (cfg_s_axi_araddr[7:0])
                    8'h04: cfg_s_axi_rdata <= reg_ctrl;
                    8'h08: cfg_s_axi_rdata <= {
                            24'h0,
                            sts_err_code, // 7:4
                            intr_pend,    // 3 - Reflects Output
                            sts_error,    // 2
                            core_busy,    // 1 - Real-time Status
                            sts_done      // 0
                        };
                    8'h0C: cfg_s_axi_rdata <= reg_src_addr;
                    8'h10: cfg_s_axi_rdata <= reg_dst_addr;
                    8'h14: cfg_s_axi_rdata <= reg_len;
                    default: begin
                        cfg_s_axi_rdata <= 32'h0;
                        cfg_s_axi_rresp <= 2'b10; // SLVERR
                    end
                endcase
            end else if (cfg_s_axi_rvalid && cfg_s_axi_rready) begin
                cfg_s_axi_rvalid <= 1'b0;
            end
        end
    end
    
    //--------------------------------------------------------------------------
    // Interrupt Logic
    //--------------------------------------------------------------------------
    // Intr = (Done || Error) && Enabled
    // Since Done/Error are sticky (W1C), Intr is inherently sticky until they are cleared.
    assign intr_pend = (sts_done || sts_error) && reg_ctrl[1];
    
    // Outputs to Core
    assign core_src_addr = reg_src_addr;
    assign core_dst_addr = reg_dst_addr;
    assign core_len      = reg_len;

endmodule
