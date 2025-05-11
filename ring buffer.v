module ring_buffer
#(
    parameter STATEWIDTH = 10,
    parameter UNIT_PER_LINE = 16,       // length of referred string: 16
    parameter ADDR_WIDTH = 10,          // window size: 2^ADDR_WIDTH=1024
    parameter BRAM_ADDR_WIDTH = 5       // 1024 / 16 / 2 = 32
)
(
    input wire sys_clk,
    input wire sys_rst_n,
    
    // signals with scanning
    input wire enable_from_scanning,
    input wire opcode_from_scanning, // 0 - write cache, 1 - append
    input wire [ADDR_WIDTH-1 : 0] addr_from_scanning,
    input wire [ADDR_WIDTH-1 : 0] endaddr_from_scanning,
    input wire [3 : 0] length_from_scanning,
    input wire [6 : 0] literal_from_scanning,
    input wire [STATEWIDTH - 1 : 0] state_from_scanning,
    output reg [STATEWIDTH - 1 : 0] state_to_scanning,

    // signals with verification
    input wire enable_from_verification,
    input wire [1 : 0] opcode_from_verification,
    input wire [ADDR_WIDTH-1 : 0] addr_from_verification,
    input wire [STATEWIDTH - 1 : 0] state_from_verification,
    output reg[2 * UNIT_PER_LINE * 7 - 1 : 0] literals_to_verification,
    output reg[2 * UNIT_PER_LINE * STATEWIDTH - 1 : 0] states_to_verification
);

// used to store source literals and states
reg[2 * UNIT_PER_LINE * 7 - 1 : 0] scanning_literals_block;
reg[2 * UNIT_PER_LINE * STATEWIDTH - 1 : 0] scanning_states_block;
reg[2 * UNIT_PER_LINE *7-1 : 0] verification_literals_block;
reg[2 * UNIT_PER_LINE*STATEWIDTH-1:0] verification_states_block;

// cache
reg[2 * UNIT_PER_LINE * 7 - 1 : 0] scanning_cache_literals;
reg[2 * UNIT_PER_LINE * STATEWIDTH - 1 : 0] scanning_cache_states;
reg[2 * UNIT_PER_LINE * STATEWIDTH - 1 : 0] verification_cache_states;
reg[ADDR_WIDTH-1 : 0] pre_scanning_cache_index;
reg[ADDR_WIDTH-1 : 0] scanning_cache_index;
reg[ADDR_WIDTH-1 : 0] pre_verification_cache_index;
reg[ADDR_WIDTH-1 : 0] verification_cache_index; 

reg[2 : 0] append_step; 
reg        write_single_step; 
reg        verification_read_step;
reg        verification_write_single_step;

// signals with BRAMs
reg scanning_enable_bram0;
reg scanning_enable_bram1;
reg scanning_write_bram0;
reg scanning_write_bram1;
reg [BRAM_ADDR_WIDTH-1 : 0] scanning_addr0;
reg [BRAM_ADDR_WIDTH-1 : 0] scanning_addr1;
reg [UNIT_PER_LINE*7-1:0] scanning_literals_to_bram0;
reg [UNIT_PER_LINE*7-1:0] scanning_literals_to_bram1;
reg [UNIT_PER_LINE*STATEWIDTH-1:0] scanning_states_to_bram0;
reg [UNIT_PER_LINE*STATEWIDTH-1:0] scanning_states_to_bram1;

reg verification_enable_bram0;
reg verification_enable_bram1;
reg verification_write_bram0;
reg verification_write_bram1;
reg [BRAM_ADDR_WIDTH-1 : 0] verification_addr0; // depth depends on UNIT_PER_LINE, when UNIT_PER_LINE is 16, depth is 32(1024 / 16 / 2)
reg [BRAM_ADDR_WIDTH-1 : 0] verification_addr1;
reg [UNIT_PER_LINE*7-1:0] verification_literals_to_bram0;
reg [UNIT_PER_LINE*7-1:0] verification_literals_to_bram1;
reg [UNIT_PER_LINE*STATEWIDTH-1:0] verification_states_to_bram0;
reg [UNIT_PER_LINE*STATEWIDTH-1:0] verification_states_to_bram1;


// signals with BRAMs
wire [UNIT_PER_LINE * 7 - 1 : 0] scanning_literals_from_bram0;
wire [UNIT_PER_LINE * 7 - 1 : 0] scanning_literals_from_bram1;
wire [UNIT_PER_LINE * STATEWIDTH - 1 : 0] scanning_states_from_bram0;
wire [UNIT_PER_LINE * STATEWIDTH - 1 : 0] scanning_states_from_bram1;


wire [UNIT_PER_LINE * 7 - 1 : 0] verification_literals_from_bram0;
wire [UNIT_PER_LINE * 7 - 1 : 0] verification_literals_from_bram1;
wire [UNIT_PER_LINE * STATEWIDTH - 1 : 0] verification_states_from_bram0;
wire [UNIT_PER_LINE * STATEWIDTH - 1 : 0] verification_states_from_bram1;


// masks
reg [2*UNIT_PER_LINE*STATEWIDTH-1:0] state_src_mask;
reg [2*UNIT_PER_LINE*STATEWIDTH-1:0] state_dst_mask_1;
reg [2*UNIT_PER_LINE*STATEWIDTH-1:0] state_dst_mask_2;
reg [2*UNIT_PER_LINE*7-1:0] literal_src_mask;
reg [2*UNIT_PER_LINE*7-1:0] literal_dst_mask_1;
reg [2*UNIT_PER_LINE*7-1:0] literal_dst_mask_2;

reg [2*UNIT_PER_LINE*7-1:0] temp_literal_cache;
reg [2*UNIT_PER_LINE*STATEWIDTH-1:0] temp_state_cache;
reg [15:0] src_move_literal;
reg [15:0] src_move_state;
reg [15:0] dst_move_literal_1;
reg [15:0] dst_move_literal_2;
reg [15:0] dst_move_state_1;
reg [15:0] dst_move_state_2;

// instantiate BRAMs here
// window size: 1024, length: 16
// BRAMs for literals: 2 x (width: 16*7, depth: 32), dual port
// BRAMs for states: 2 x (width: 16 * STATEWIDTH, depth: 32), dual port


always@(posedge sys_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        // initialization
        append_step <= 3'b000;
        scanning_enable_bram0 <= 0;
        scanning_enable_bram1 <= 0;
        verification_enable_bram0 <= 0;
        verification_enable_bram1 <= 0;
        scanning_cache_index <= 0;
        verification_cache_index <= 0;
        write_single_step <= 0;
        pre_scanning_cache_index <= 0;
        pre_verification_cache_index <= 0;
        verification_read_step <= 0;
        verification_write_single_step <= 0; 
    end
    else begin
        // if(enable_from_scanning) begin
        //     // handle scanning module's request
        //     if(opcode_from_scanning == 1'b0) begin
        //         case(write_single_step)
        //             1'b0: begin
        //                 // write cache
        //                 scanning_enable_bram0 <= 0;
        //                 scanning_enable_bram1 <= 0;
        //                 verification_enable_bram0 <= 0;
        //                 verification_enable_bram1 <= 0;
        //                 pre_scanning_cache_index <= scanning_cache_index;
        //                 scanning_cache_index <= scanning_cache_index + 1;
        //                 scanning_cache_literals[((scanning_cache_index % (2*UNIT_PER_LINE)) + 1) * 7 - 1 -: 7] <= literal_from_scanning;
        //                 scanning_cache_states[((scanning_cache_index % (2*UNIT_PER_LINE)) + 1) * STATEWIDTH - 1 -: STATEWIDTH] <= state_from_scanning;
        //                 write_single_step <= 1;
        //             end
        //             1'b1: begin
        //                 // one of the cache blocks is full, flush it to bram
        //                 if(pre_scanning_cache_index[4:0] == 15) begin // depends on UNIT_PER_LINE
        //                     if(addr_from_scanning[4] == 0) begin
        //                         // flush to bram0
        //                         scanning_enable_bram0 <= 1;
        //                         scanning_write_bram0 <= 1;
        //                         scanning_addr0 <= addr_from_scanning[ADDR_WIDTH -1 -: BRAM_ADDR_WIDTH];
        //                         scanning_literals_to_bram0 <= scanning_cache_literals[UNIT_PER_LINE*7-1:0];
        //                         scanning_states_to_bram0 <= scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];
        //                     end
        //                     else begin
        //                         // flush to bram1                            
        //                         scanning_enable_bram1 <= 1;
        //                         scanning_write_bram1 <= 1;
        //                         scanning_addr1 <= addr_from_scanning[ADDR_WIDTH -1 -: BRAM_ADDR_WIDTH];
        //                         scanning_literals_to_bram1 <= scanning_cache_literals[UNIT_PER_LINE*7-1:0];
        //                         scanning_states_to_bram1 <= scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];
        //                     end
        //                 end
        //                 if(pre_scanning_cache_index[4:0] == 31) begin
        //                     if(addr_from_scanning[4] == 0) begin
        //                         // flush to bram0
        //                         scanning_enable_bram0 <= 1;
        //                         scanning_write_bram0 <= 1;
        //                         scanning_addr0 <= addr_from_scanning[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH];
        //                         scanning_literals_to_bram0 <= scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7];
        //                         scanning_states_to_bram0 <= scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
        //                     end
        //                     else begin
        //                         // flush to bram1                            
        //                         scanning_enable_bram1 <= 1;
        //                         scanning_write_bram1 <= 1;
        //                         scanning_addr1 <= addr_from_scanning[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH];
        //                         scanning_literals_to_bram1 <= scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7];
        //                         scanning_states_to_bram1 <= scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
        //                     end
        //                 end
        //                 write_single_step <= 0;
        //             end
        //         endcase
        //     end
        //     else begin
        //         // append referred string and its states to cache, cache[cache_index +: len] <= block[addr +: len]
        //         case(append_step)
        //             3'd0: begin
        //                 // compute bram address
        //                 scanning_enable_bram0 <= 1'b1;
        //                 scanning_enable_bram1 <= 1'b1;
        //                 scanning_write_bram0 <= 1'b0;
        //                 scanning_write_bram1 <= 1'b0;
        //                 // addr_from_scanning / UNIT_PER_LINE % 2
        //                 if(addr_from_scanning[4] == 0) begin
        //                     scanning_addr0 <= addr_from_scanning[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH];
        //                     scanning_addr1 <= addr_from_scanning[ADDR_WIDTH-1  -: BRAM_ADDR_WIDTH];
        //                 end
        //                 else begin
        //                     scanning_addr0 <= addr_from_scanning[ADDR_WIDTH -1 -: BRAM_ADDR_WIDTH] + 1;
        //                     scanning_addr1 <= addr_from_scanning[ADDR_WIDTH -1 -: BRAM_ADDR_WIDTH];
        //                 end
        //                 // start to prepare mask
        //                 src_move_literal <= (2*UNIT_PER_LINE - length_from_scanning) * 7;
        //                 src_move_state <= (2*UNIT_PER_LINE - length_from_scanning) * STATEWIDTH;
        //                 dst_move_literal_1 <= (length_from_scanning + (scanning_cache_index % (2*UNIT_PER_LINE))) * 7;
        //                 dst_move_literal_2 <= (2*UNIT_PER_LINE - (scanning_cache_index % (2*UNIT_PER_LINE))) * 7;
        //                 dst_move_state_1 <= (length_from_scanning + (scanning_cache_index % (2*UNIT_PER_LINE))) * STATEWIDTH;
        //                 dst_move_state_2 <= (2*UNIT_PER_LINE - scanning_cache_index % (2*UNIT_PER_LINE)) * STATEWIDTH; 
        //                 append_step <= 3'd1;
        //             end
        //             3'd1: begin
        //                 // bram returned data, combine block
        //                 if(scanning_cache_index / UNIT_PER_LINE == addr_from_scanning / UNIT_PER_LINE) begin
        //                     // first blcok hits cache
        //                     if(addr_from_scanning[4] == 0) begin
        //                         if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
        //                             // block = {_from_bram1, cache[UNIT_PER_LINE*7-1:0]}
        //                             scanning_literals_block <= {scanning_literals_from_bram1, scanning_cache_literals[UNIT_PER_LINE*7-1:0]};
        //                             scanning_states_block <= {scanning_states_from_bram1, scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0]};
        //                         end
        //                         else begin
        //                             scanning_literals_block <= {scanning_literals_from_bram1, scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7]};
        //                             scanning_states_block <= {scanning_states_from_bram1, scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH]};
        //                         end
        //                     end
        //                     else begin
        //                         // block = {_from_block0, cache[]}
        //                         if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
        //                             scanning_literals_block <= {scanning_literals_from_bram0, scanning_cache_literals[UNIT_PER_LINE*7-1:0]};
        //                             scanning_states_block <= {scanning_states_from_bram0, scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0]};
        //                         end
        //                         else begin
        //                             scanning_literals_block <= {scanning_literals_from_bram0, scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7]};
        //                             scanning_states_block <= {scanning_states_from_bram0, scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH]};
        //                         end                               
        //                     end
        //                 end
        //                 else if(scanning_cache_index / UNIT_PER_LINE == addr_from_scanning / UNIT_PER_LINE + 1) begin
        //                     // second block hits cache, block = {cache[] , from_bram}
        //                     if(addr_from_scanning[4] == 0) begin
        //                         if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
        //                             scanning_literals_block <= {scanning_cache_literals[UNIT_PER_LINE*7-1:0], scanning_literals_from_bram0};
        //                             scanning_states_block <= {scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0], scanning_states_from_bram0};
        //                         end
        //                         else begin
        //                             scanning_literals_block <= {scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7], scanning_literals_from_bram0};
        //                             scanning_states_block <= {scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH], scanning_states_from_bram0};
        //                         end
        //                     end
        //                     else begin
        //                         if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
        //                             scanning_literals_block <= {scanning_cache_literals[UNIT_PER_LINE*7-1:0], scanning_literals_from_bram1};
        //                             scanning_states_block <= {scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0], scanning_states_from_bram1};
        //                         end
        //                         else begin
        //                             scanning_literals_block <= {scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7], scanning_literals_from_bram1};
        //                             scanning_states_block <= {scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH], scanning_states_from_bram1};
        //                         end                              
        //                     end
        //                 end
        //                 else begin
        //                     // cache missed 
        //                     if(addr_from_scanning[4] == 0) begin
        //                         scanning_literals_block <= {scanning_literals_from_bram1, scanning_literals_from_bram0};
        //                         scanning_states_block <= {scanning_states_from_bram1, scanning_states_from_bram0};
        //                     end
        //                     else begin
        //                         scanning_literals_block <= {scanning_literals_from_bram0, scanning_literals_from_bram1};
        //                         scanning_states_block <= {scanning_states_from_bram0, scanning_states_from_bram1};
        //                     end
        //                 end
        //                 // last state hits cache
        //                 if(endaddr_from_scanning / UNIT_PER_LINE == scanning_cache_index / UNIT_PER_LINE) begin
        //                     state_to_scanning <= scanning_cache_states[(endaddr_from_scanning % (2*UNIT_PER_LINE) + 1) * STATEWIDTH - 1 -: STATEWIDTH];
        //                 end
        //                 else begin
        //                     if(endaddr_from_scanning[4] == 0) begin
        //                         state_to_scanning <= scanning_states_from_bram0[(endaddr_from_scanning % UNIT_PER_LINE + 1) * STATEWIDTH - 1 -: STATEWIDTH];
        //                     end
        //                     else begin
        //                         state_to_scanning <= scanning_states_from_bram1[(endaddr_from_scanning % UNIT_PER_LINE + 1) * STATEWIDTH - 1 -: STATEWIDTH];
        //                     end
        //                 end

        //                 literal_src_mask <= {2*UNIT_PER_LINE*7{1'b1}} << src_move_literal;
        //                 state_src_mask <= {2*UNIT_PER_LINE*STATEWIDTH{1'b1}} << src_move_state;
        //                 literal_dst_mask_1 <= {2*UNIT_PER_LINE*7{1'b1}} << dst_move_literal_1;
        //                 literal_dst_mask_2 <= {2*UNIT_PER_LINE*7{1'b1}} >> dst_move_literal_2;
        //                 state_dst_mask_1 <= {2*UNIT_PER_LINE*STATEWIDTH{1'b1}} << dst_move_state_1;
        //                 state_dst_mask_2 <= {2*UNIT_PER_LINE*STATEWIDTH{1'b1}} >> dst_move_state_2;
        //                 append_step <= 3'd2;
        //             end
        //             3'd2: begin
        //                 // 
        //                 scanning_literals_block <= scanning_literals_block << src_move_literal;
        //                 scanning_states_block <= scanning_states_block << src_move_state;
        //                 state_dst_mask_1 <= state_dst_mask_1 | state_dst_mask_2;
        //                 literal_dst_mask_1 <= literal_dst_mask_1 | literal_dst_mask_2;
        //                 append_step <= 3'd3;
        //             end
        //             3'd3: begin
        //                 scanning_literals_block <= scanning_literals_block & literal_src_mask;
        //                 scanning_states_block <= scanning_states_block & state_src_mask;
        //                 scanning_cache_states <= scanning_cache_states & state_dst_mask_1;
        //                 scanning_cache_literals <= scanning_cache_literals & literal_dst_mask_2;
        //                 append_step <= 3'd4;
        //             end 
        //             3'd4: begin
        //                 src_move_literal <= (2*UNIT_PER_LINE - length_from_scanning - scanning_cache_index % (2*UNIT_PER_LINE)) * 7;
        //                 src_move_state <= (2*UNIT_PER_LINE - length_from_scanning - scanning_cache_index % (2*UNIT_PER_LINE)) * STATEWIDTH;
        //                 append_step <= 3'd5;
        //             end
        //             3'd5: begin
        //                 scanning_literals_block <= scanning_literals_block >> src_move_literal;
        //                 scanning_states_block <= scanning_states_block >> src_move_state;
        //                 append_step <= 3'd6;
        //             end
        //             3'd6: begin
        //                 scanning_cache_literals <= scanning_cache_literals | scanning_literals_block;
        //                 scanning_cache_states <= scanning_cache_states | scanning_states_block;
        //                 scanning_cache_index <= scanning_cache_index + length_from_scanning;
        //                 pre_scanning_cache_index <= scanning_cache_index;
        //                 append_step <= 3'd7;
        //             end
        //             3'd7: begin
        //                 if(pre_scanning_cache_index / UNIT_PER_LINE != scanning_cache_index / UNIT_PER_LINE) begin
        //                     if(pre_scanning_cache_index[4] == 0) begin
        //                         scanning_enable_bram0 <= 1'b1;
        //                         scanning_write_bram0 <= 1'b1;
        //                         if(pre_scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
        //                             scanning_literals_to_bram0 <= scanning_cache_literals[UNIT_PER_LINE*7-1:0];
        //                             scanning_states_to_bram0 <= scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];   
        //                         end
        //                         else begin
        //                             scanning_literals_to_bram0 <= scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7];
        //                             scanning_states_to_bram0 <= scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
        //                         end
        //                     end
        //                     else begin
        //                         scanning_enable_bram1 <= 1'b1;
        //                         scanning_write_bram1 <= 1'b1;
        //                         if(pre_scanning_cache_index % (2*UNIT_PER_LINE) <= UNIT_PER_LINE) begin
        //                             scanning_literals_to_bram1 <= scanning_cache_literals[UNIT_PER_LINE*7-1:0];
        //                             scanning_states_to_bram1 <= scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];   
        //                         end
        //                         else begin
        //                             scanning_literals_to_bram1 <= scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7];
        //                             scanning_states_to_bram1 <= scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];                            
        //                         end
        //                     end
        //                 end
        //                 append_step <= 3'd0;
        //             end
        //         endcase
        //     end
        // end
        if(enable_from_verification) begin
            // handle verification module's request
            case(opcode_from_verification)
                2'd0: begin
                    case(verification_read_step)
                        1'b0: begin
                            verification_enable_bram0 <= 1'b1;
                            verification_enable_bram1 <= 1'b1;
                            verification_write_bram0 <= 1'b0;
                            verification_write_bram1 <= 1'b0;
                            if(addr_from_verification[4] == 0) begin
                                verification_addr0 <= addr_from_verification[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH];
                                verification_addr1 <= addr_from_verification[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH];
                            end
                            else begin
                                verification_addr0 <= addr_from_verification[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH] + 1;
                                verification_addr1 <= addr_from_verification[ADDR_WIDTH-1 - : BRAM_ADDR_WIDTH];
                            end
                            verification_read_step <= 1'b1;
                        end
                        1'b1: begin
                            // brams returned literals and states
                            verification_enable_bram0 <= 1'b0;
                            verification_enable_bram1 <= 1'b0;
                            if(addr_from_verification / UNIT_PER_LINE == scanning_cache_index / UNIT_PER_LINE) begin
                                if(addr_from_verification[4] == 0) begin
                                    // to verification: {bram part, cache part}
                                    if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
                                        states_to_verification <= {verification_states_from_bram1, scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0]};
                                        verification_cache_states <= {verification_states_from_bram1, scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0]};
                                        literals_to_verification <= {verification_literals_from_bram1, scanning_cache_literals[UNIT_PER_LINE*7-1:0]};
                                    end
                                    else begin
                                        states_to_verification <= {verification_states_from_bram1, scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH]};
                                        verification_cache_states <= {verification_states_from_bram1, scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH]};
                                        literals_to_verification <= {verification_literals_from_bram1, scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7]}; 
                                    end
                                end
                                else begin
                                    if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
                                        states_to_verification <= {verification_states_from_bram0, scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0]};
                                        verification_cache_states <= {verification_states_from_bram0, scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0]};
                                        literals_to_verification <= {verification_literals_from_bram0, scanning_cache_literals[UNIT_PER_LINE*7-1:0]};
                                    end
                                    else begin
                                        states_to_verification <= {verification_states_from_bram0, scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH]};
                                        verification_cache_states <= {verification_states_from_bram0, scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH]};
                                        literals_to_verification <= {verification_literals_from_bram0, scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7]}; 
                                    end
                                end
                            end
                            else if(addr_from_verification / UNIT_PER_LINE + 1 == scanning_cache_index / UNIT_PER_LINE) begin
                                if(addr_from_verification[4] == 0) begin
                                    // to verification: {cache part, bram part}
                                    if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
                                        states_to_verification <= {scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0], verification_states_from_bram0};
                                        verification_cache_states <= {scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0], verification_states_from_bram0};
                                        literals_to_verification <= {scanning_cache_literals[UNIT_PER_LINE*7-1:0], verification_literals_from_bram0};
                                    end
                                    else begin
                                        states_to_verification <= {scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH], verification_states_from_bram0};
                                        verification_cache_states <= {scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH], verification_states_from_bram0};
                                        literals_to_verification <= {scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7], verification_literals_from_bram0};   
                                    end
                                end
                                else begin
                                    if(scanning_cache_index % (2*UNIT_PER_LINE) <= 15) begin
                                        states_to_verification <= {scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0], verification_states_from_bram1};
                                        verification_cache_states <= {scanning_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0], verification_states_from_bram1};
                                        literals_to_verification <= {scanning_cache_literals[UNIT_PER_LINE*7-1:0], verification_literals_from_bram1};
                                    end
                                    else begin
                                        states_to_verification <= {scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH], verification_states_from_bram1};
                                        verification_cache_states <= {scanning_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH], verification_states_from_bram1};
                                        literals_to_verification <= {scanning_cache_literals[2*UNIT_PER_LINE*7-1:UNIT_PER_LINE*7], verification_literals_from_bram1};   
                                    end
                                end
                            end
                            else begin
                                if(addr_from_verification[4] == 0) begin
                                    states_to_verification <= {verification_states_from_bram1, verification_states_from_bram0};
                                    verification_cache_states <= {verification_states_from_bram1, verification_states_from_bram0};
                                    literals_to_verification <= {verification_literals_from_bram1, verification_literals_from_bram0};
                                end
                                else begin
                                    states_to_verification <= {verification_states_from_bram0, verification_states_from_bram1};
                                    verification_cache_states <= {verification_states_from_bram0, verification_states_from_bram1};
                                    literals_to_verification <= {verification_literals_from_bram0, verification_literals_from_bram1};
                                end
                            end
                        end
                    endcase
                end
                2'd1: begin
                    // write single state
                    if(addr_from_verification / UNIT_PER_LINE != scanning_cache_index / UNIT_PER_LINE) begin
                        case(verification_write_single_step) 
                            1'b0: begin
                                verification_enable_bram0 <= 0;
                                verification_enable_bram1 <= 0;
                                verification_cache_states[(addr_from_verification % (2*UNIT_PER_LINE) + 1) * STATEWIDTH - 1 -: STATEWIDTH] <= state_from_verification;
                                verification_cache_index <= addr_from_verification + 1;
                                pre_verification_cache_index <= addr_from_verification;
                                verification_write_single_step <= 1'b1;
                            end
                            1'b1: begin
                                if(pre_verification_cache_index % (2*UNIT_PER_LINE) == 15) begin
                                    if(pre_verification_cache_index[4] == 0) begin
                                        verification_enable_bram0 <= 1'b1;
                                        verification_write_bram0 <= 1'b1;
                                        verification_addr0 <= pre_verification_cache_index[ADDR_WIDTH - 1 -: BRAM_ADDR_WIDTH];
                                        verification_states_to_bram0 <= verification_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];
                                    end
                                    else begin
                                        verification_enable_bram1 <= 1'b1;
                                        verification_write_bram1 <= 1'b1;
                                        verification_addr1 <= pre_verification_cache_index[ADDR_WIDTH - 1 -: BRAM_ADDR_WIDTH];
                                        verification_states_to_bram1 <= verification_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];
                                    end
                                end
                                if(pre_verification_cache_index % (2*UNIT_PER_LINE) == 31) begin
                                    if(pre_verification_cache_index[4] == 0) begin
                                        verification_enable_bram0 <= 1'b1;
                                        verification_write_bram0 <= 1'b1;
                                        verification_addr0 <= pre_verification_cache_index[ADDR_WIDTH - 1 -: BRAM_ADDR_WIDTH];
                                        verification_states_to_bram0 <= verification_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
                                    end
                                    else begin
                                        verification_enable_bram1 <= 1'b1;
                                        verification_write_bram1 <= 1'b1;
                                        verification_addr1 <= pre_verification_cache_index[ADDR_WIDTH - 1 -: BRAM_ADDR_WIDTH];
                                        verification_states_to_bram1 <= verification_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
                                    end 
                                end
                                verification_write_single_step <= 1'b0;
                            end
                        endcase
                    end
                end
                2'd2: begin
                    // flush v-cache
                    if(scanning_cache_index / UNIT_PER_LINE != addr_from_verification / UNIT_PER_LINE) begin
                        if(addr_from_verification[4] == 0) begin
                            verification_enable_bram0 <= 1'b1;
                            verification_write_bram0 <= 1'b1;
                            verification_addr0 <= addr_from_verification[ADDR_WIDTH-1 -: BRAM_ADDR_WIDTH];
                            if(addr_from_verification % (2*UNIT_PER_LINE) <= 15) begin
                                verification_states_to_bram0 <= verification_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];
                            end
                            else begin
                                verification_states_to_bram0 <= verification_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
                            end
                        end
                        else begin
                            verification_enable_bram1 <= 1'b1;
                            verification_write_bram1 <= 1'b1;
                            verification_addr1 <= addr_from_verification[ADDR_WIDTH - 1 -: BRAM_ADDR_WIDTH];
                            if(addr_from_verification % (2*UNIT_PER_LINE) <= 15) begin
                                verification_states_to_bram1 <= verification_cache_states[UNIT_PER_LINE*STATEWIDTH-1:0];
                            end
                            else begin
                                verification_states_to_bram1 <= verification_cache_states[2*UNIT_PER_LINE*STATEWIDTH-1:UNIT_PER_LINE*STATEWIDTH];
                            end
                        end
                    end
                end 
            endcase
        end
    end
end
endmodule