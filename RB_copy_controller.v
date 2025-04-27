

module RB_copy_controller (
    input  wire        clk,      // 时钟信号
    input  wire        rst_n,    // 低电平有效的复位信号

	// BRAM接口

	output reg [15:0] BRAM_read_addr,
	input [258*8-1:0] BRAM_read_data,

	output reg [15:0] BRAM_write_addr,
	output reg [258*8-1:0] BRAM_write_data,
	output reg BRAM_write_en,

	

	// 状态机接口
	input copy_en,
	input [15:0] copy_BRAM_addr,
	input [9:0] copy_offset,
	input [9:0] copy_length,

	input [15:0] past_BRAM_addr,
	input [9:0] past_offset,
	input [9:0] past_length

);

    // 状态编码，使用参数定义便于修改
    parameter [2:0] IDLE    = 3'b000,
                   READ_BRAM_SRC   = 3'b001,
                   READ_BRAM_SRC1   = 3'b010,
                   GET_SRC_and_READ_BRAM_DST   = 3'b011,
		GET_SRC1   = 3'b100,
		GET_DST   = 3'b101,
		GENOUTPUT   = 3'b110,
		STATE7   = 3'b111;



	// 自用变量
	reg [258*8*2-1:0] MASK;
	reg [258*8*2-1:0] temp_src_block;
	reg [258*8*2-1:0] temp_dst_block;
	reg [15:0] copy_BRAM_addr_reg;
	reg [9:0] copy_offset_reg;
	reg [9:0] copy_length_reg;
	reg [15:0] past_BRAM_addr_reg;
	reg [9:0] past_offset_reg;
	reg [9:0] past_length_reg;


    // 状态寄存器，只需要定义当前状态
    reg [2:0] current_state;

    // 单个always块，包含状态转移和输出逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位条件
            current_state <= IDLE;
            MASK <= {258*8*2{1'b1}};
            temp_src_block <= 258*8*2'h0;
            temp_dst_block <= 258*8*2'h0;
        end
        else begin
            case (current_state)
                IDLE: begin
                    
                    if (copy_en) begin
                        copy_BRAM_addr_reg <= copy_BRAM_addr;
                        copy_offset_reg <= copy_offset;
                        copy_length_reg <= copy_length;
			past_BRAM_addr_reg <= past_BRAM_addr;
			past_offset_reg <= past_offset;
			past_length_reg <= past_length;

                        current_state <= READ_BRAM_SRC;
                    end
                end

                READ_BRAM_SRC: begin
                                         BRAM_read_addr <= copy_BRAM_addr_reg;
		                         MASK <= MASK << (past_offset_reg*8);

		                         current_state <= READ_BRAM_SRC1;
                end

                READ_BRAM_SRC1: begin
                                        BRAM_read_addr <= copy_BRAM_addr_reg + 16'b1;
		                        MASK <= MASK >> (258*8*2 - copy_length_reg*8);

		                        current_state <= GET_SRC_and_READ_BRAM_DST;
                end

                GET_SRC_and_READ_BRAM_DST: begin
                                        BRAM_read_addr <= dst_BRAM_addr_reg;
		                        temp_src_block <= {BRAM_read_data,258*8'h0}
		                        MASK <= MASK << (258*8*2 - copy_length_reg*8 - dst_offset_reg*8);

		                        current_state <= GET_SRC1;
                end

                GET_SRC1: begin
					temp_src_block <= {temp_src_block[258*8*2-1:258*8],BRAM_read_data}
					
					current_state <= GET_DST;
                end

                GET_DST: begin
					temp_dst_block <= {BRAM_read_data,258*8'h0}
					temp_src_block <= ((temp_src_block << copy_offset_reg*8) >> dst_offset_reg*8) & MASK;
					current_state <= GENOUTPUT;
                end

                GENOUTPUT: begin
                                        temp_dst_block <= (temp_dst_block & ~MASK) | temp_src_block;


					if(copy_en) begin
						current_state <= ???;
					end
					else begin
						current_state <= ???;
					end
                end


                default: begin
                    current_state <= IDLE;
                    output1 <= 2'b00;
                end
            endcase
        end
    end

endmodule
