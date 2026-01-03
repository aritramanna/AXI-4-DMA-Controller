`timescale 1ns/1ps

////////////////////////////////////////////////////////////////////////////////
// Module Name: axi_dma_master
// Description:
//    - AXI4 Master DMA controller for high-bandwidth memory-to-memory transfers.
//    - Supports single-burst transfers up to 4KB (256 beats x 128-bit).
//    - Features independent source/destination watchdog timers.
//    - Handles 4KB boundary checks and alignment validation.
//    - Uses an internal FWFT FIFO for elastic buffering.
////////////////////////////////////////////////////////////////////////////////

module axi_dma_master #(
    parameter AXI_ADDR_W = 32,
    parameter AXI_DATA_W = 128,  // 128-bit for high bandwidth
    parameter AXI_ID_W   = 4,
    parameter FIFO_DEPTH = 256,  // Exact fit for 4KB (256 beats)
    parameter TIMEOUT_SRC_CYCLES = 128, // Source Read Timeout cycles
    parameter TIMEOUT_DST_CYCLES = 128  // Destination Write Timeout cycles
)(
    // Clock and Reset
    input  logic                    clk,
    input  logic                    rst_n,
    
    // DMA Control Interface
    input  logic                    dma_start,
    input  logic [AXI_ADDR_W-1:0]   dma_src_addr,
    input  logic [AXI_ADDR_W-1:0]   dma_dst_addr,
    input  logic [31:0]             dma_length,      // In bytes (max 4096)
    output logic                    dma_done,
    output logic [3:0]              dma_completion_status,  // 4-bit Error Code (0=OK)
    output logic                    dma_busy,
    
    // AXI4 Read Address Channel
    output logic [AXI_ID_W-1:0]     axi_arid,
    output logic [AXI_ADDR_W-1:0]   axi_araddr,
    output logic [7:0]              axi_arlen,
    output logic [2:0]              axi_arsize,
    output logic [1:0]              axi_arburst,
    output logic                    axi_arvalid,
    input  logic                    axi_arready,
    
    // AXI4 Read Data Channel
    input  logic [AXI_ID_W-1:0]     axi_rid,
    input  logic [AXI_DATA_W-1:0]   axi_rdata,
    input  logic [1:0]              axi_rresp,
    input  logic                    axi_rlast,
    input  logic                    axi_rvalid,
    output logic                    axi_rready,
    
    // AXI4 Write Address Channel
    output logic [AXI_ID_W-1:0]     axi_awid,
    output logic [AXI_ADDR_W-1:0]   axi_awaddr,
    output logic [7:0]              axi_awlen,
    output logic [2:0]              axi_awsize,
    output logic [1:0]              axi_awburst,
    output logic                    axi_awvalid,
    input  logic                    axi_awready,
    
    // AXI4 Write Data Channel
    output logic [AXI_DATA_W-1:0]   axi_wdata,
    output logic [AXI_DATA_W/8-1:0] axi_wstrb,
    output logic                    axi_wlast,
    output logic                    axi_wvalid,
    input  logic                    axi_wready,
    
    // AXI4 Write Response Channel
    input  logic [AXI_ID_W-1:0]     axi_bid,
    input  logic [1:0]              axi_bresp,
    input  logic                    axi_bvalid,
    output logic                    axi_bready
);

    //==========================================================================
    // Local Parameters
    //==========================================================================
    localparam BYTES_PER_BEAT = AXI_DATA_W / 8;  // 16 bytes
    localparam AXSIZE         = $clog2(BYTES_PER_BEAT);  // 4 (for 16 bytes)

`ifdef COCOTB_SIM
    initial begin
        $dumpfile("axi_dma_dump.vcd");
        $dumpvars(0, axi_dma_master);
    end
`endif
    
    // AXI Response Codes
    localparam AXI_RESP_OKAY  = 2'b00;

    // Error Type Definition
    typedef enum logic [3:0] {
        ERR_NONE        = 4'h0,
        ERR_ALIGN_SRC   = 4'h1,  // Unaligned Source Address
        ERR_ALIGN_DST   = 4'h2,  // Unaligned Dest Address
        ERR_ALIGN_LEN   = 4'h3,  // Length not multiple of 16
        ERR_ZERO_LEN    = 4'h4,  // Zero Length
        ERR_4K_SRC      = 4'h5,  // Source crosses 4KB boundary
        ERR_4K_DST      = 4'h6,  // Dest crosses 4KB boundary
        ERR_LEN_LARGE   = 4'h7,  // Length > 4096
        ERR_TIMEOUT_SRC = 4'h8,  // Source Read Timeout
        ERR_TIMEOUT_DST = 4'h9,  // Destination Write Timeout
        ERR_AXI_RESP    = 4'hF   // AXI Slave Error (RRESP/BRESP)
    } dma_err_t;
    
    // State Machine
    typedef enum logic [2:0] {
        IDLE        = 3'b000,  // Waiting for DMA start
        CHECK_PARAMS= 3'b001,  // Validate addresses and length
        RD_ADDR     = 3'b010,  // Issuing read address
        RD_DATA     = 3'b011,  // Receiving read data
        WR_ADDR     = 3'b100,  // Issuing write address
        WR_DATA     = 3'b101,  // Sending write data
        WR_RESP     = 3'b110,  // Waiting for write response
        DONE        = 3'b111   // Transfer complete/Error
    } state_t;

    //==========================================================================
    // Registered Signals (FSM ONLY updates these)
    //==========================================================================
    
    // State
    state_t state;
    
    // DMA Configuration (captured on start)
    logic [AXI_ADDR_W-1:0] src_addr;          // Read address
    logic [AXI_ADDR_W-1:0] dst_addr;          // Write address
    logic [31:0] target_len;                  // Transfer length
    
    // Burst Tracking
    logic [7:0] burst_cnt;  // Beat counter within burst
    
    // Status
    dma_err_t error_code;   // Sticky error code

    // TIMEOUT COUNTERS
    logic [31:0] src_timer;
    logic [31:0] dst_timer;

    //==========================================================================
    // Combinational Signals
    //==========================================================================
    
    // Error tracking
    logic axi_error;
    
    // Burst length (calculated from dma_length)
    logic [7:0] burst_len;

    // Parameter Check Signals (Internal to FSM now)
    // Removed external wires
    
    // FIFO Interface
    logic                  fifo_wr_en;
    logic [AXI_DATA_W-1:0] fifo_din;
    logic                  fifo_rd_en;
    logic [AXI_DATA_W-1:0] fifo_dout;
    logic                  fifo_empty;

    // FIFO Reset Logic
    logic fifo_soft_rst;
    // Reset FIFO when in DONE state (cleans up after errors/aborts)
    // or continuously in IDLE (optional, but DONE covers the post-xfer case)
    assign fifo_soft_rst = (state == DONE);

    //==========================================================================
    // FIFO Instantiation
    //==========================================================================
    fifo_bram_fwft #(
        .DATA_W (AXI_DATA_W),
        .DEPTH  (FIFO_DEPTH)
    ) u_fifo (
        .clk    (clk),
        .rst_n  (rst_n && !fifo_soft_rst), // Active low reset
        .wr_en  (fifo_wr_en),
        .din    (fifo_din),
        .rd_en  (fifo_rd_en),
        .dout   (fifo_dout),
        .empty  (fifo_empty)
    );

    //==========================================================================
    // Combinational Logic
    //==========================================================================
    
    // Error detection (any non-OKAY response)
    assign axi_error = (axi_rvalid && axi_rready && axi_rresp != AXI_RESP_OKAY) ||
                       (axi_bvalid && axi_bready && axi_bresp != AXI_RESP_OKAY);

    //==========================================================================
    // Burst Length Calculation
    //==========================================================================
    // Burst length (calculated from registered length)
    // Constraints: target_len is multiple of 16 and <= 4096
    assign burst_len = target_len[11:4] - 1'b1;

    //==========================================================================
    // Main State Machine (Simplified Single-Burst)
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all registered signals
            state       <= IDLE;
            src_addr    <= '0;
            dst_addr    <= '0;
            burst_cnt   <= '0;
            error_code  <= ERR_NONE;
            src_timer   <= '0;
            dst_timer   <= '0;
        end else begin
            
            //==================================================================
            // State Machine
            //==================================================================
            case (state)
                //--------------------------------------------------------------
                // IDLE: Wait for DMA start command
                //--------------------------------------------------------------
                IDLE: begin
                    if (dma_start) begin
                        // Capture configuration
                        src_addr   <= dma_src_addr;
                        dst_addr   <= dma_dst_addr;
                        target_len <= dma_length;
                        state      <= CHECK_PARAMS;
                        error_code <= ERR_NONE;  // Clear error when starting new xfer

                        src_timer  <= '0;
                        dst_timer  <= '0;
                    end
                end

                //--------------------------------------------------------------
                // CHECK_PARAMS: Validate Constraints
                //--------------------------------------------------------------
                CHECK_PARAMS: begin
                    // Perform all checks on registered values
                    
                    // 1. Check Source Address Alignment (must be 16-byte aligned)
                    if (src_addr[3:0] != 4'b0) begin
                        error_code <= ERR_ALIGN_SRC;
                        state      <= DONE;
                    
                    // 2. Check Destination Address Alignment (must be 16-byte aligned)
                    end else if (dst_addr[3:0] != 4'b0) begin
                        error_code <= ERR_ALIGN_DST;
                        state      <= DONE;
                    
                    // 3. Check Length Alignment (must be multiple of 16 bytes)
                    end else if (target_len[3:0] != 4'b0) begin
                        error_code <= ERR_ALIGN_LEN;
                        state      <= DONE;
                    
                    // 4. Check for Zero Length (invalid transfer)
                    end else if (target_len == 0) begin
                        error_code <= ERR_ZERO_LEN;
                        state      <= DONE;
                    
                    // 5. Check if Length > 4096
                    end else if (target_len > 4096) begin
                        error_code <= ERR_LEN_LARGE;
                        state      <= DONE;

                    // 6. Check if Source Transfer Crosses 4KB Boundary
                    //    (Start Address[31:12] must equal End Address[31:12])
                    end else if (src_addr[31:12] != (src_addr + target_len - 1'b1) >> 12) begin
                        error_code <= ERR_4K_SRC;
                        state      <= DONE;
                    
                    // 7. Check if Destination Transfer Crosses 4KB Boundary
                    end else if (dst_addr[31:12] != (dst_addr + target_len - 1'b1) >> 12) begin
                        error_code <= ERR_4K_DST;
                        state      <= DONE;
                    
                    // All checks passed -> Proceed to Read Address Phase
                    end else begin
                        state      <= RD_ADDR;
                    end
                end
                
                //--------------------------------------------------------------
                // RD_ADDR: Issue read address
                //--------------------------------------------------------------
                RD_ADDR: begin
                    // Wait for address handshake
                    if (axi_arvalid && axi_arready) begin
                        state <= RD_DATA;
                    end
                end
                
                //--------------------------------------------------------------
                // RD_DATA: Receive read data beats
                //--------------------------------------------------------------
                RD_DATA: begin
                    // Wait for last beat (slave provides axi_rlast)
                    if (axi_rvalid && axi_rready && axi_rlast) begin
                        state <= WR_ADDR;
                    end
                end
                
                //--------------------------------------------------------------
                // WR_ADDR: Issue write address
                //--------------------------------------------------------------
                WR_ADDR: begin
                    // Wait for address handshake
                    if (axi_awvalid && axi_awready) begin
                        burst_cnt <= '0;  // Reset beat counter
                        state     <= WR_DATA;
                    end
                end
                
                //--------------------------------------------------------------
                // WR_DATA: Send write data beats
                //--------------------------------------------------------------
                WR_DATA: begin
                    // Handle write handshake
                    if (axi_wvalid && axi_wready) begin
                        burst_cnt <= burst_cnt + 1'b1;
                        
                        // Wait for last beat, then get response
                        if (axi_wlast) begin
                            state <= WR_RESP;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // WR_RESP: Wait for write response
                //--------------------------------------------------------------
                WR_RESP: begin
                    if (axi_bvalid && axi_bready) begin
                        state <= DONE;  // Transfer complete check
                    end
                end

                //--------------------------------------------------------------
                // DONE: Assert dma_done and wait/return to IDLE
                //--------------------------------------------------------------
                DONE: begin
                    // Unconditional return to IDLE (creates 1-cycle pulse)
                    // Or wait for !dma_start if handshake requires it.
                    // For now, auto-transition to IDLE.
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
            
            //==================================================================
            // Source Timeout Logic (Watchdog)
            //==================================================================
            if (state == RD_ADDR) begin
                if (axi_arvalid && !axi_arready) begin
                    src_timer <= src_timer + 1;
                    if (src_timer > TIMEOUT_SRC_CYCLES) begin
                        state <= DONE;
                        error_code <= ERR_TIMEOUT_SRC;
                    end
                end else begin
                    src_timer <= '0; // Reset on handshake
                end
            end else if (state == RD_DATA) begin
                 if (axi_rready && !axi_rvalid) begin
                    src_timer <= src_timer + 1;
                    if (src_timer > TIMEOUT_SRC_CYCLES) begin
                        state <= DONE;
                        error_code <= ERR_TIMEOUT_SRC;
                    end
                 end else begin
                    src_timer <= '0; // Reset on handshake
                 end
            end else begin
                src_timer <= '0;
            end

            //==================================================================
            // Destination Timeout Logic (Watchdog)
            //==================================================================
            if (state == WR_ADDR) begin
                if (axi_awvalid && !axi_awready) begin
                    dst_timer <= dst_timer + 1;
                    if (dst_timer > TIMEOUT_DST_CYCLES) begin
                        state <= DONE;
                        error_code <= ERR_TIMEOUT_DST;
                    end
                end else begin
                    dst_timer <= '0;
                end
            end else if (state == WR_DATA) begin
                if (axi_wvalid && !axi_wready) begin
                    dst_timer <= dst_timer + 1;
                    if (dst_timer > TIMEOUT_DST_CYCLES) begin
                        state <= DONE;
                        error_code <= ERR_TIMEOUT_DST;
                    end
                end else begin
                    dst_timer <= '0;
                end
            end else if (state == WR_RESP) begin
                if (axi_bready && !axi_bvalid) begin
                    dst_timer <= dst_timer + 1;
                    if (dst_timer > TIMEOUT_DST_CYCLES) begin
                        state <= DONE;
                        error_code <= ERR_TIMEOUT_DST;
                    end
                end else begin
                    dst_timer <= '0;
                end
            end else begin
                dst_timer <= '0;
            end
            
            //==================================================================
            // Error Detection (sticky flag for AXI errors)
            //==================================================================
            if (state != IDLE && axi_error)
                error_code <= ERR_AXI_RESP;
        end
    end

    //==========================================================================
    // Output Assignments (Combinational - State-Based)
    //==========================================================================
    always_comb begin
        // Default assignments (prevent latches)
        axi_rready  = 1'b0;
        axi_bready  = 1'b0;
        fifo_wr_en  = 1'b0;
        fifo_rd_en  = 1'b0;

        // Default AXI output assignments
        axi_arid    = '0;
        axi_araddr  = '0;
        axi_arlen   = '0;
        axi_arsize  = '0;
        axi_arburst = '0;
        axi_arvalid = 1'b0;

        axi_awid    = '0;
        axi_awaddr  = '0;
        axi_awlen   = '0;
        axi_awsize  = '0;
        axi_awburst = '0;
        axi_awvalid = 1'b0;

        axi_wdata   = '0;
        axi_wstrb   = '0;
        axi_wlast   = 1'b0;
        axi_wvalid  = 1'b0;
        
        // State-dependent outputs
        case (state)
            RD_ADDR: begin
                // Read address channel signals
                axi_arid    = '0;
                axi_araddr  = src_addr;
                axi_arlen   = burst_len;
                axi_arsize  = AXSIZE[2:0];
                axi_arburst = 2'b01;  // INCR
                axi_arvalid = 1'b1;
            end
            
            RD_DATA: begin
                // Accept read data
                axi_rready = 1'b1;
                // Write to FIFO on handshake
                fifo_wr_en = axi_rvalid && axi_rready;
            end
            
            WR_ADDR: begin
                // Write address channel signals
                axi_awid    = '0;
                axi_awaddr  = dst_addr;
                axi_awlen   = burst_len;
                axi_awsize  = AXSIZE[2:0];
                axi_awburst = 2'b01;  // INCR
                axi_awvalid = 1'b1;
            end
            
            WR_DATA: begin
                // Write Data Validity
                axi_wvalid = !fifo_empty;

                // Read from FIFO when handshake occurs
                // Only pop if Slave is ready AND we have data
                fifo_rd_en = axi_wvalid && axi_wready;

                // Write data signals
                axi_wdata  = fifo_dout;
                axi_wstrb  = '1;
                axi_wlast  = (burst_cnt == burst_len);
            end
            
            WR_RESP: begin
                // Accept write response
                axi_bready = 1'b1;
            end
        endcase
        
        // Signals used across multiple states or always active
        
        // FIFO Write Data
        fifo_din = axi_rdata;
        
        
        // Output Assignments
    end
    assign dma_done = (state == DONE);
    assign dma_busy = (state != IDLE);
    assign dma_completion_status = error_code;

endmodule
