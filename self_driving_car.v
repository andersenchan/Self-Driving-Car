module main(KEY, LEDR, CLOCK_50, GPIO_0, SW, 
    HEX0, HEX1, HEX2, HEX3, HEX4, HEX5);
	
	input CLOCK_50;
	input [9:0] SW;
	input [3:0]KEY;
    input [5] GPIO_0;
    
	output [3:0] LEDR;
	output [4:0] GPIO_0;
	output [6] GPIO_0;
	
	output [6:0] HEX0;
	output [6:0] HEX1;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX4;
	output [6:0] HEX5;
	
	wire l_motor_hi, l_motor_lo, r_motor_hi, r_motor_lo;

	reg [31:0] distance;

    control c(
        .up(~KEY[2]),
        .down(~KEY[1]),
        .left(~KEY[3]),
        .right(~KEY[0]),
        .l_motor_hi(l_motor_hi),
		.l_motor_lo(l_motor_lo),
        .r_motor_hi(r_motor_hi),
		.r_motor_lo(r_motor_lo),
		.speed(SW[8:7]),
        .clk(CLOCK_50),
        .reset(SW[9])
    );
	
	assign GPIO_0[0] = r_motor_lo;
	assign GPIO_0[1] = l_motor_lo;
	assign GPIO_0[2] = r_motor_hi;
	assign GPIO_0[3] = l_motor_hi;
	
	assign LEDR[0] = r_motor_hi;
	assign LEDR[1] = r_motor_lo;
	assign LEDR[2] = l_motor_hi;
	assign LEDR[3] = l_motor_lo;

	ultrasonic u0(
	    .trig(GPIO_0[4]),
	    .echo(GPIO_0[5]),
	    .clk(CLOCK_50),
	    .reset(SW[9]),
	    .wall_sense(GPIO_0[6]),
	    .distance(distance[31:0])
	);

	hex_decoder h0(distance[3:0], HEX0[6:0]);
	hex_decoder h1(distance[7:4], HEX1[6:0]);
	hex_decoder h2(distance[11:8], HEX2[6:0]);
	hex_decoder h3(distance[15:12], HEX3[6:0]);
	hex_decoder h4(distance[19:16], HEX4[6:0]);
	hex_decoder h5(distance[23:20], HEX5[6:0]);
    
endmodule

module ultrasonic(trig, echo, clk, reset, wall_sense, distance);

    input echo;
    input clk;
    input reset;
    output trig;
    output wall_sense;

    output reg [31:0] distance;
    reg [8:0] trig_time;
    reg [21:0] echo_time; // 21bits long is the max because of the timeout
    reg [21:0] timeout;

	always @(posedge clock)
    begin
    	if (reset)
    		begin
    			trig_time <= 0;
    			echo_time <= 0;
    			trig <=0;
    			timeout <= 0;
    			wall_sense <= 0;
    		end

    	if (trig_time < 9'b111110101) // trig for 10us, 50 cycles per us, wait for counter to hit 501
    		begin
    			trig_time <= trig_time + 1b'1;
    			trig <= 1;
    			wall_sense <= 0;
    		end

    	if (echo)
    		begin
    			echo_time <= echo_time + 1b'1;
    		end

    	if (trig_time >= 9'b111110101) // trig was on for 10us
    		begin
    			trig <=0;
    			timeout <= timeout + 1b'1; // count to 60ms (3 million cycles), then timeout if echo hasn't been recieved
    			if (timeout == 22'b1011011100011011000000)
    				begin
    					trig_time <= 0;
    					echo_time <= 0;
    					timeout <= 0;
    				end
    		end

    	if ((echo_time > 1) && (~echo)) // if echo is done emitting
    		begin
	   		distance <= echo_time * (0.02) * (0.01724137931) // num of cycles x (1us/50cycles) x (1us/58) = centimeters
	   		if (distance < 3) // if less than 3 cm
	   			begin
	   				wall_sense <= 1;
	   			end
    			trig_time <= 0;
    			echo_time <= 0;
    			trig <=0;
    			timeout <= 0;
    			//wall_sense <= 0;
    			end
    		end
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule

module control(up, down, left, right, 
    l_motor_hi, l_motor_lo, r_motor_hi, r_motor_lo, speed, reset, clk);

    input up;
    input down;
    input left;
    input right;
    input [1:0] speed;
    input reset;
    input clk;
    
    output reg l_motor_hi;
	output reg l_motor_lo;
	output reg r_motor_hi;
    output reg r_motor_lo;
    
    reg [3:0] direction;
    assign direction[0] = right;
    assign direction[1] = left;
    assign direction[2] = down;
    assign direction[3] = up;
    
    reg [3:0] current_state;
    localparam  forward      = 4'd0,
                reverse      = 4'd1,
                turn_left    = 4'd2,
                turn_right   = 4'd3,
                rotate_left  = 4'd4,
                rotate_right = 4'd5,
                curve_left   = 4'd6,
                curve_right  = 4'd7,
				idle         = 4'd8;
	
	reg [31:0] timeout;
	
	// Speed initialization
	always@(*)
	begin
	    case(speed)
	        2'b0: timeout <= 32'd32;
	        2'b1: timeout <= 32'd16;
	        2'b2: timeout <= 32'd8;
	        2'b3: timeout <= 32'd4;
	    endcase
	end
    
    // State Table  
    always@(*)
    begin
        case (direction)
            4'd0:  current_state <= idle;
            4'd1:  current_state <= rotate_right;
            4'd2:  current_state <= rotate_left;
            4'd3:  current_state <= idle;
            4'd4:  current_state <= reverse;
            4'd5:  current_state <= curve_right;
            4'd6:  current_state <= curve_left;
            4'd7:  current_state <= reverse;
            4'd8:  current_state <= forward;
            4'd9:  current_state <= turn-right;
            4'd10: current_state <= turn_left;
            4'd11: current_state <= forward;
            4'd12: current_state <= idle;
            4'd13: current_state <= rotate_right;
            14'd4: current_state <= rotate_left;
            4'd15: current_state <= idle;
    		default: current_state <= idle;
		endcase
    end
    
    // Output logic    
    always@(*)
    begin 
        l_motor_hi <= 0;
        r_motor_hi <= 0;
        l_motor_lo <= 0;
        r_motor_lo <= 0;

        if (speed_counter >= timeout) begin // since timeout can change
            case(current_state)
                forward: begin
                    l_motor_hi <= 1;
    				l_motor_lo <= 0;
                    r_motor_hi <= 1;
                    r_motor_lo <= 0;
                    end
                reverse: begin
                    l_motor_hi <= 0;
    				l_motor_lo <= 1;
                    r_motor_hi <= 0;
                    r_motor_lo <= 1;
                    end
                turn_left: begin
                    l_motor_hi <= 0;
    				l_motor_lo <= 0;
                    r_motor_hi <= 1;
                    r_motor_lo <= 0;
                    end
                turn_right: begin
                    l_motor_hi <= 1;
    				l_motor_lo <= 0;
                    r_motor_hi <= 0;
                    r_motor_lo <= 0;
                    end
    			rotate_left: begin
                    l_motor_hi <= 0;
    				l_motor_lo <= 1;
                    r_motor_hi <= 1;
                    r_motor_lo <= 0;
                    end
                rotate_right: begin
                    l_motor_hi <= 1;
    				l_motor_lo <= 0;
                    r_motor_hi <= 0;
                    r_motor_lo <= 1;
                    end
                curve_left: begin
                    l_motor_hi <= 0;
    				l_motor_lo <= 0;
                    r_motor_hi <= 0;
                    r_motor_lo <= 1;
                    end
                curve_right: begin
                    l_motor_hi <= 0;
    				l_motor_lo <= 1;
                    r_motor_hi <= 0;
                    r_motor_lo <= 0;
                    end
    			idle: begin
    				l_motor_hi <= 0;
    				r_motor_hi <= 0;
    				l_motor_lo <= 0;
    				r_motor_lo <= 0;
    				end
                //no default case needed
    		endcase
    	end
    end
    
    //Counter (setup and update)
    always@(*) 
    begin
        if (reset || speed_counter >= timeout) // since timeout can change
            speed_counter = 0;
        else
            speed_counter <= speed_counter + 1'b1;
    end
    
    
endmodule
