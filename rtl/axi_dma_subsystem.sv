`timescale 1ns/1ps

////////////////////////////////////////////////////////////////////////////////
// Module Name: axi_dma_subsystem
// Description: Top-Level Wrapper for AXI4 DMA Subsystem
////////////////////////////////////////////////////////////////////////////////

module axi_dma_subsystem #(
    parameter AXI_ADDR_W = 32,
    parameter AXI_DATA_W = 128,
    parameter AXI_ID_W   = 4,
    // Core Parameters
    parameter FIFO_DEPTH = 256,
    parameter TIMEOUT_SRC_CYCLES = 100000,
    parameter TIMEOUT_DST_CYCLES = 100000
)(
    input  logic                    clk,
    input  logic                    rst_n,
    
    // AXI4-Lite Slave Interface (Control)
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

    // AXI4 Master Interface (Data)
    output logic [AXI_ID_W-1:0]     m_axi_arid,
    output logic [AXI_ADDR_W-1:0]   m_axi_araddr,
    output logic [7:0]              m_axi_arlen,
    output logic [2:0]              m_axi_arsize,
    output logic [1:0]              m_axi_arburst,
    output logic                    m_axi_arvalid,
    input  logic                    m_axi_arready,
    input  logic [AXI_ID_W-1:0]     m_axi_rid,
    input  logic [AXI_DATA_W-1:0]   m_axi_rdata,
    input  logic [1:0]              m_axi_rresp,
    input  logic                    m_axi_rlast,
    input  logic                    m_axi_rvalid,
    output logic                    m_axi_rready,
    output logic [AXI_ID_W-1:0]     m_axi_awid,
    output logic [AXI_ADDR_W-1:0]   m_axi_awaddr,
    output logic [7:0]              m_axi_awlen,
    output logic [2:0]              m_axi_awsize,
    output logic [1:0]              m_axi_awburst,
    output logic                    m_axi_awvalid,
    input  logic                    m_axi_awready,
    output logic [AXI_DATA_W-1:0]   m_axi_wdata,
    output logic [AXI_DATA_W/8-1:0] m_axi_wstrb,
    output logic                    m_axi_wlast,
    output logic                    m_axi_wvalid,
    input  logic                    m_axi_wready,
    input  logic [AXI_ID_W-1:0]     m_axi_bid,
    input  logic [1:0]              m_axi_bresp,
    input  logic                    m_axi_bvalid,
    output logic                    m_axi_bready,

    // Interrupt Output (Pending Status)
    output logic                    intr_pend
);

    //==========================================================================
    // Internal Signals
    //==========================================================================
    logic        core_start;
    logic [31:0] core_src_addr;
    logic [31:0] core_dst_addr;
    logic [31:0] core_len;
    logic        core_done;
    logic [3:0]  core_status;
    logic        core_busy;
    

    //==========================================================================
    // Register Block Instantiation
    //==========================================================================
    dma_reg_block #(
        .AXI_ADDR_W(AXI_ADDR_W)
    ) u_regs (
        .clk(clk),
        .rst_n(rst_n),
        // AXIL
        .cfg_s_axi_awaddr(cfg_s_axi_awaddr),
        .cfg_s_axi_awvalid(cfg_s_axi_awvalid),
        .cfg_s_axi_awready(cfg_s_axi_awready),
        .cfg_s_axi_wdata(cfg_s_axi_wdata),
        .cfg_s_axi_wstrb(cfg_s_axi_wstrb),
        .cfg_s_axi_wvalid(cfg_s_axi_wvalid),
        .cfg_s_axi_wready(cfg_s_axi_wready),
        .cfg_s_axi_bresp(cfg_s_axi_bresp),
        .cfg_s_axi_bvalid(cfg_s_axi_bvalid),
        .cfg_s_axi_bready(cfg_s_axi_bready),
        .cfg_s_axi_araddr(cfg_s_axi_araddr),
        .cfg_s_axi_arvalid(cfg_s_axi_arvalid),
        .cfg_s_axi_arready(cfg_s_axi_arready),
        .cfg_s_axi_rdata(cfg_s_axi_rdata),
        .cfg_s_axi_rresp(cfg_s_axi_rresp),
        .cfg_s_axi_rvalid(cfg_s_axi_rvalid),
        .cfg_s_axi_rready(cfg_s_axi_rready),
        // Core Control
        .core_start(core_start),
        .core_src_addr(core_src_addr),
        .core_dst_addr(core_dst_addr),
        .core_len(core_len),
        .core_done(core_done),
        .core_busy(core_busy), 
        .core_status(core_status),
        // Interrupt
        .intr_pend(intr_pend)
    );

    //==========================================================================
    // Core Instantiation
    //==========================================================================
    axi_dma_master #(
        .AXI_ADDR_W(AXI_ADDR_W),
        .AXI_DATA_W(AXI_DATA_W),
        .AXI_ID_W(AXI_ID_W),
        .FIFO_DEPTH(FIFO_DEPTH),
        .TIMEOUT_SRC_CYCLES(TIMEOUT_SRC_CYCLES),
        .TIMEOUT_DST_CYCLES(TIMEOUT_DST_CYCLES)
    ) u_core (
        .clk(clk),
        .rst_n(rst_n),
        .dma_start(core_start),
        .dma_src_addr(core_src_addr),
        .dma_dst_addr(core_dst_addr),
        .dma_length(core_len),
        .dma_done(core_done),
        .dma_completion_status(core_status),
        .dma_busy(core_busy),
        // Master Interface
        .axi_arid(m_axi_arid),
        .axi_araddr(m_axi_araddr),
        .axi_arlen(m_axi_arlen),
        .axi_arsize(m_axi_arsize),
        .axi_arburst(m_axi_arburst),
        .axi_arvalid(m_axi_arvalid),
        .axi_arready(m_axi_arready),
        .axi_rid(m_axi_rid),
        .axi_rdata(m_axi_rdata),
        .axi_rresp(m_axi_rresp),
        .axi_rlast(m_axi_rlast),
        .axi_rvalid(m_axi_rvalid),
        .axi_rready(m_axi_rready),
        .axi_awid(m_axi_awid),
        .axi_awaddr(m_axi_awaddr),
        .axi_awlen(m_axi_awlen),
        .axi_awsize(m_axi_awsize),
        .axi_awburst(m_axi_awburst),
        .axi_awvalid(m_axi_awvalid),
        .axi_awready(m_axi_awready),
        .axi_wdata(m_axi_wdata),
        .axi_wstrb(m_axi_wstrb),
        .axi_wlast(m_axi_wlast),
        .axi_wvalid(m_axi_wvalid),
        .axi_wready(m_axi_wready),
        .axi_bid(m_axi_bid),
        .axi_bresp(m_axi_bresp),
        .axi_bvalid(m_axi_bvalid),
        .axi_bready(m_axi_bready)
    );

endmodule
