function setInitialStance

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );
clear all
%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(5, 1);   % connect to port 0, at 1 MBaud
pause(2);
motor_ids = [12,14,16, 11,13,15];      % list of motor IDs to command
load('Motor_Biases.mat');
Motor_Biases = -1* motor_bias;
%% Test Case - Trajectory: Goal Position/Speed
motor_ids_initial =[ 1:5,9,10, 17:22];
bias = [zeros(1,5), Motor_Biases(1:2)',Motor_Biases(9:12)',0,0]'  ;
%bias = zeros(13,1);
dxl_io.set_compliance_margin(motor_ids_initial, zeros(13,1), 'cw');
dxl_io.set_motor_pos_speed(motor_ids_initial,bias,...
    zeros(length(motor_ids_initial),1), ones(length(motor_ids_initial),1)*pi/20);
pause(1);

%motor 6 is 90 degrees
dxl_io.set_motor_pos_speed(6,0,-pi/2,pi/6);

%motor 7
dxl_io.set_motor_pos_speed(7,0,pi/4,pi/6);
pause(0.1);
%motor 8
dxl_io.set_motor_pos_speed(8,0,-pi/4,pi/6);
pause(0.1);
%motor_bias =zeros(6,1);
%motor_bias =[-1;4;-2;4;-2;2];
motor_bias =[Motor_Biases(4:2:8); Motor_Biases(3:2:7)];
%motor_bias = Motor_Biases';
dxl_io.set_compliance_margin(motor_ids, [1;1;1;1;1;1], 'cw');
dxl_io.set_motor_pos_speed(motor_ids,motor_bias, ...
 [ 0; 0  ;  0;   0;        0 ;0], ones(length(motor_ids),1)*pi/6);
% dxl_io.set_motor_pos_speed(20,0, ...
%  pi/4, pi/6);

end