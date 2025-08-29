
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;      

entity census_transformer is
    generic(
        data_width : integer := 8; --size of input pixels
        census_width : integer := 24; -- comparisons in a 5x5 window
        window_size : integer := 5;
        line_width : integer :=  1280; --size of a line of the image
        N_lines : integer := 720
    );
    port(
        clk : in std_logic;
        reset : in std_logic;
        pixel_in : in std_logic_vector(data_width - 1  downto 0);
        data_valid : in std_logic; -- valid input
        census_out : out std_logic_vector(census_width - 1 downto 0);
        valid_out : out std_logic
    );
end entity;    

architecture behav of census_transformer is
    
    type u_mat_t is array(natural range <>, natural range <>) of unsigned(data_width - 1  downto 0); --matrix of unsigned
    type slv_arr_8bit is array(natural range <>) of std_logic_vector(data_width - 1 downto 0); 
    type slv_arr_12bit is array(natural range <>) of std_logic_vector(10 downto 0); 
    type fsm_t is(SLEEP, VALID, NOT_VALID);


    component line_buffer_FIFO is         
        port(
            clk : IN STD_LOGIC;
            srst : IN STD_LOGIC;
            din : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
            wr_en : IN STD_LOGIC;
            rd_en : IN STD_LOGIC;
            dout : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
            full : OUT STD_LOGIC;
            empty : OUT STD_LOGIC;
            valid : OUT STD_LOGIC;
            data_count : OUT STD_LOGIC_VECTOR(10 DOWNTO 0);
            wr_rst_busy : OUT STD_LOGIC;
            rd_rst_busy : OUT STD_LOGIC
        );
    end component;

    --constants to control the read_flags
    constant PADDED_IMG_SIZE : integer := line_width + 4;
    constant THRESH_1 : integer := 13 + PADDED_IMG_SIZE - 1; 
    constant THRESH_2 : integer := 13 + 2*(PADDED_IMG_SIZE - 2);
    constant THRESH_3 : integer := 13 + 3*(PADDED_IMG_SIZE - 2) - 1;
    constant THRESH_4 : integer := 13 + 4*(PADDED_IMG_SIZE - 2) - 2 ;
    constant NOT_VALID_CYCLES : integer :=  3;

    signal data_counts : slv_arr_12bit(0 to window_size - 1); --array of the 
    signal window : u_mat_t (0 to window_size - 1, 0 to window_size - 1); --window buffer
    signal fifo_outputs : slv_arr_8bit(0 to window_size - 1);   --array of the outputs of the line buffers 
    signal read_flags : std_logic_vector(0 to window_size - 1); --array of the read_enable bits
    signal valid_arr : std_logic_vector(0 to 3); --arrays of the valid signals of the line buffers
    signal start_output_frame : std_logic;
    signal start_up_counter : integer ; 
    signal line_counter : integer;
    signal value_counter_valid : integer; --counts the valid values
    signal value_counter_NOT_valid : integer; --counts the cycles during not valid state output
    signal curr_state, next_state, prev_state : fsm_t ;

begin
    
    --istances of the line buffers 
    g_LINE_BUFFERS : for i in 0 to window_size - 1 generate
        g_FIRST : if i = 0 generate
            i_FIFO : line_buffer_FIFO 
                port map(
                    clk => clk,
                    srst => reset,
                    din => pixel_in,
                    wr_en => data_valid,
                    rd_en => read_flags(i),
                    dout => fifo_outputs(i),
                    data_count => data_counts(i),
                    valid => valid_arr(i),
                    full => open,
                    empty => open,
                    wr_rst_busy => open,
                    rd_rst_busy => open                
                    );
        end generate;
        g_MID : if i > 0 and i < window_size - 1  generate
            i_FIFO : line_buffer_FIFO 
                port map(
                    clk => clk,
                    srst => reset,
                    din => fifo_outputs(i - 1),
                    wr_en => valid_arr(i - 1),
                    rd_en => read_flags(i),
                    dout => fifo_outputs(i),
                    valid => valid_arr(i),
                    data_count => data_counts(i),
                    full => open,
                    empty => open,
                    wr_rst_busy => open,
                    rd_rst_busy => open                
                    );
        end generate;
        g_LAST : if i = window_size - 1 generate
            i_FIFO : line_buffer_FIFO 
                port map(
                    clk => clk,
                    srst => reset,
                    din => fifo_outputs(i - 1),
                    wr_en => valid_arr(i - 1),
                    rd_en => read_flags(i),
                    dout => fifo_outputs(i),
                    data_count => data_counts(i),
                    valid => open,
                    full => open,
                    empty => open,
                    wr_rst_busy => open,
                    rd_rst_busy => open                
                    );
        end generate;
    end generate;

    --window register -- datas shifts through the rows of the window
    p_WINDOW : process(clk) begin
        if rising_edge(clk) then
            if reset = '1' then
                for i in 0 to window_size - 1 loop
                    for j in 0 to window_size - 1 loop
                        window(i, j) <= to_unsigned(0, data_width);
                    end loop; 
                end loop;
            else
                -- first rows are linked with the line_buffers
               for j in 0 to window_size - 1  loop
                    window(j, 0) <= unsigned(fifo_outputs(j));
               end loop; 
                -- datas shifts through the window rows
                for i in 1 to window_size - 1 loop
                    for j in 0 to window_size - 1 loop
                        window(j, i) <= window(j, i - 1);
                    end loop;
                end loop;
            end if;
        end if;
    end process;
    
     -- logic to compute the output census value from the window content
     p_CENSUS : process(window) 
        variable central_pixel : unsigned(data_width - 1 downto 0) := to_unsigned(0, data_width);
        variable census_value : std_logic_vector(census_width - 1 downto 0) := (others => '0'); 
        variable bit_index : integer := 0;
    begin
         central_pixel := window((window_size - 1) / 2, (window_size - 1) / 2);
         bit_index := 0;
         for i in 0 to window_size - 1 loop
             for j in 0 to window_size - 1 loop
                 if i = ((window_size - 1) / 2)  and j = ((window_size - 1) / 2) then
                    next;
                 else
                     if central_pixel > window(i, j) then
                         census_value(bit_index) := '0';
                     else
                    census_value(bit_index) := '1';
                     end if;
                     bit_index := bit_index + 1;
                 end if;
         end loop;       
         end loop;
         census_out <= census_value;
    end process;



    --logic to handle the reading of the line buffers
    p_READ_FLAGS : process(start_up_counter) begin 
        if start_up_counter < 13 then
            read_flags <= "11111";
        elsif start_up_counter >= 13 and start_up_counter < THRESH_1 then --fill line buffer 4
            read_flags <= "11110" ;
        elsif start_up_counter >= THRESH_1 and start_up_counter < THRESH_2 then --fill line buffer 3
            read_flags <= "11100" ;
        elsif start_up_counter >= THRESH_2 and start_up_counter < THRESH_3 then --fill line buffer 2
            read_flags <= "11000" ;
        elsif start_up_counter >= THRESH_3 and start_up_counter < THRESH_4 then --fill line buffer 1
            read_flags <= "10000";
        else 
            read_flags <= "11111"; --end of the 
        end if;
    end process;


    --start_up_counter that is triggered from the arrival of the fist pixel
    p_COUNTER : process(clk) begin
        if rising_edge(clk) then
            if reset = '1' then
                start_up_counter <= 0;
            else
                if data_valid = '1' or curr_state /= NOT_VALID then 
                    start_up_counter <= start_up_counter + 1;
                else
                    start_up_counter <= 0;
                end if;
            end if;
        end if;
    end process;

    start_output_frame <= '1' when start_up_counter = THRESH_4 + 6 else '0';

-----state machine to control the valid_out signal

    p_STATE_MEMORY : process(clk) begin --state memory
        if rising_edge(clk) then 
            if reset = '1' then
                curr_state <= SLEEP;
            else
                curr_state <= next_state;
                prev_state <= curr_state;
            end if;
        end if;
    end process;

    p_NOT_VALUE_COUNTER : process(clk) begin --counts the cycles during VALID state
        if rising_edge(clk) then 
            if reset = '1' then
                value_counter_valid <= 0;
            else
                if curr_state = VALID then
                    value_counter_valid <= value_counter_valid + 1;
                else
                    value_counter_valid <= 0;
                end if;
            end if;
        end if;
    end process;


    p_VALUE_COUNTER : process(clk) begin --counts the cycles during NOT VALID state
        if rising_edge(clk) then 
            if reset = '1' then
                value_counter_NOT_valid <= 0;
            else
                if curr_state = NOT_VALID then
                    value_counter_NOT_valid <= value_counter_NOT_valid + 1;
                else
                    value_counter_NOT_valid <= 0;
                end if;
            end if;
        end if;
    end process;

    p_LINE_COUNTER : process(clk, reset) begin  --counts the lines produced in output
        if rising_edge(clk) then 
            if reset = '1' then
                line_counter <= 0;
            else
                if curr_state = NOT_VALID and prev_state <= VALID then
                    line_counter <= line_counter + 1;
                elsif curr_state  = SLEEP then
                    line_counter <= 0;
                else
                    line_counter <= line_counter;
                end if;
            end if;
        end if;
    end process;

    p_NEXT_STATE_LOGIC : process(curr_state, line_counter,start_output_frame, value_counter_valid, value_counter_NOT_valid) begin
        valid_out <= '0';
        case curr_state is
            when SLEEP =>
                if start_output_frame = '1' then
                    next_state <= VALID;
                else
                    next_state <= SLEEP;
                end if;
            
            when VALID =>
                valid_out <= '1';
                if value_counter_valid = line_width - 1 then
                    if line_counter = N_lines - 1 then
                        next_state <= SLEEP;
                    else
                        next_state <= NOT_VALID;
                    end if;
                else
                    next_state <= VALID;
                end if;

            when NOT_VALID =>
                if value_counter_NOT_valid = NOT_VALID_CYCLES then
                    next_state <= VALID;
                else
                    next_state  <= NOT_VALID;
                end if;
            
            end case;
    end process;

end architecture;