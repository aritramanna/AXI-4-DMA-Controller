`timescale 1ns/1ps

module fifo_bram_fwft #(
    parameter DATA_W = 32,
    parameter DEPTH  = 1024
)(
    input  logic              clk,
    input  logic              rst_n,
    input  logic              wr_en,
    input  logic [DATA_W-1:0] din,
    input  logic              rd_en,
    output logic              full,
    output logic [DATA_W-1:0] dout,
    output logic              empty
);

    //==========================================================================
    // Local Parameters
    //==========================================================================
    localparam PTR_W = $clog2(DEPTH);
    localparam CNT_W = $clog2(DEPTH + 3);  // BRAM + Middle + Skid

    //==========================================================================
    // Signal Declarations
    //==========================================================================
    
    // BRAM Storage
    logic [DATA_W-1:0] mem [0:DEPTH-1];
    
    // Pointers
    logic [PTR_W-1:0] wr_ptr;
    logic [PTR_W-1:0] rd_ptr;
    
    // Pipeline Registers
    logic [DATA_W-1:0] mem_q;        // Middle stage data
    logic              mem_q_valid;  // Middle stage valid
    logic [DATA_W-1:0] skid_data;    // Output stage data
    logic              skid_valid;   // Output stage valid
    
    // Count (total items in FIFO)
    logic [CNT_W-1:0] count;
    
    // Combinational Control Signals
    logic ram_empty;
    logic ram_wr_en;
    logic ram_read_en;
    logic skid_taking_data;
    logic user_wr_effective;
    logic user_rd_effective;

    //==========================================================================
    // Combinational Control Logic
    //==========================================================================
    
    // RAM Empty: BRAM is empty when all data is in pipeline
    assign ram_empty = (count == (mem_q_valid + skid_valid));
    
    // Skid Transfer: Move Middle→Skid when Middle valid AND Skid ready
    assign skid_taking_data = mem_q_valid && (!skid_valid || rd_en);
    
    // Prefetch: Read BRAM if not empty AND pipeline needs data
    assign ram_read_en = !ram_empty && (!mem_q_valid || skid_taking_data);
    
    // Write Enable: Write if user requests and not full
    assign ram_wr_en = wr_en && !full;
    
    // User Effective Actions (for count updates)
    // Throughput optimization: Allow write when full IF read is clearing space simultaneously
    assign user_wr_effective = wr_en && (!full || (rd_en && !empty));
    assign user_rd_effective = rd_en && !empty;
    
    // Output Assignments
    assign dout = skid_data;
    assign empty = !skid_valid;  // Original logic - works with AXI DMA
    assign full = (count >= DEPTH);  // Full when count reaches or exceeds DEPTH

    //==========================================================================
    // Sequential Logic (Single always_ff block)
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all state
            wr_ptr      <= '0;
            rd_ptr      <= '0;
            count       <= '0;
            mem_q       <= '0;
            mem_q_valid <= 1'b0;
            skid_data   <= '0;
            skid_valid  <= 1'b0;
        end else begin
            
            //==================================================================
            // 1. BRAM Write Operation
            //==================================================================
            if (ram_wr_en) begin
                mem[wr_ptr] <= din;
                wr_ptr <= wr_ptr + 1'b1;
            end
            
            //==================================================================
            // 2. BRAM Read Operation (1-cycle latency)
            //==================================================================
            if (ram_read_en) begin
                mem_q  <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1'b1;
            end
            
            //==================================================================
            // 3. Middle Stage Valid Control
            //==================================================================
            if (ram_read_en) begin
                mem_q_valid <= 1'b1;
            end else if (skid_taking_data) begin
                mem_q_valid <= 1'b0;
            end
            
            //==================================================================
            // 4. Skid Buffer Updates (PRIORITY LOGIC)
            //==================================================================
            // Refill has priority over pop
            if (skid_taking_data) begin
                skid_data  <= mem_q;
                skid_valid <= 1'b1;
            end else if (rd_en && skid_valid) begin
                skid_valid <= 1'b0;
            end
            
            //==================================================================
            // 5. Count Updates (Total Items)
            //==================================================================
            case ({user_wr_effective, user_rd_effective})
                2'b10:   count <= count + 1'b1;  // Write only
                2'b01:   count <= count - 1'b1;  // Read only
                default: count <= count;          // Simultaneous or neither
            endcase
        end
    end

endmodule
