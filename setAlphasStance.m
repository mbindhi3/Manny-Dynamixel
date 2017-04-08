function setAlphasStance

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );
clear all
%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(5, 1);   % connect to port 0, at 1 MBaud

motor_ids = [12,14,16, 11,13,15];      % list of motor IDs to command

%% Test Case - Trajectory: Goal Position/Speed
motor_ids_initial =[ 1:5,10, 17:22];

dxl_io.set_motor_pos_speed(motor_ids_initial, zeros(length(motor_ids_initial), ...
    1), zeros(length(motor_ids_initial),1), ones(length(motor_ids_initial),1)*pi/20);
pause(1);


%motor 9 is reversed
dxl_io.set_motor_pos_speed(9,1,pi/1.9,pi/6);

%motor 6 is 90 degrees
dxl_io.set_motor_pos_speed(6,1,-pi/2,pi/6);

%motor 7
dxl_io.set_motor_pos_speed(7,1,pi/4,pi/6);
pause(0.1);
%motor 8
dxl_io.set_motor_pos_speed(8,1,-pi/4,pi/6);
pause(0.1);

alphaTraj = 'leftCyclePart2Alphas.mat';
load(alphaTraj);
a =leftCyclePart2Alphas;
dxl_io.set_motor_pos_speed(motor_ids,zeros(length(motor_ids), 1), ...
   [a(1,end); -a(2,end); a(3,end);-a(4,end); a(5,end); -a(6,end)], ones(length(motor_ids),1)*pi/6);

end