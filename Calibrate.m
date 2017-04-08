%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );

%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(5, 1);   % connect to port 0, at 1 MBaud

motor_ids = [12,14,16, 11,13,15];      % list of motor IDs to command
load('biases_left.mat');
load('extra_left.mat');
%% Test Case - Trajectory: Goal Position/Speed
motor_ids_initial =[ 1:5,10, 17:22];
% motor_bias = dxl_io.calibrate_motor(10, 1 )
% extra_biases= [extra_biases motor_bias];

% dxl_io.set_motor_pos_speed(10, extra_biases(1), ...
%      0, pi/20);
%  
% motor_bias = dxl_io.calibrate_motor(20, 1 )
% extra_biases= [extra_biases motor_bias];
% 
% dxl_io.set_motor_pos_speed(20, motor_bias,0, pi/20);
%   
% motor_bias = dxl_io.calibrate_motor(12, 1 )
% Motor_Biases= [Motor_Biases motor_bias];
% 
% dxl_io.set_motor_pos_speed(12, Motor_Biases, ...
%      0, pi/20);
% motor_bias = dxl_io.calibrate_motor(14, 1 )
% Motor_Biases= [Motor_Biases motor_bias];
% dxl_io.set_motor_pos_speed(14, Motor_Biases(2), ...
%      0, pi/20);
% 
% motor_bias = dxl_io.calibrate_motor(16, 1 )
% Motor_Biases= [Motor_Biases motor_bias]
% 
% dxl_io.set_motor_pos_speed(16, Motor_Biases(3), ...
%      0, pi/20);
% 

% motor_bias = dxl_io.calibrate_motor(18, 1 )
% extra_biases= [extra_biases motor_bias];
% 
% dxl_io.set_motor_pos_speed(18, extra_biases(3), ...
%      0, pi/20);
 
motor_bias = dxl_io.calibrate_motor(19, 1 )
extra_biases= [extra_biases motor_bias];

dxl_io.set_motor_pos_speed(19, motor_bias,0, pi/20);
  
motor_bias = dxl_io.calibrate_motor(11, 1 )
Motor_Biases= [Motor_Biases motor_bias]; 
dxl_io.set_motor_pos_speed(11, Motor_Biases(4), ...
     0, pi/20);
 
motor_bias = dxl_io.calibrate_motor(13, 1 )
Motor_Biases= [Motor_Biases motor_bias];

dxl_io.set_motor_pos_speed(13, Motor_Biases(5), ...
     0, pi/20);
 
motor_bias = dxl_io.calibrate_motor(15, 1 )
Motor_Biases= [Motor_Biases motor_bias];

dxl_io.set_motor_pos_speed(15, Motor_Biases(6), ...
     0, pi/20);
 
motor_bias = dxl_io.calibrate_motor(17, 1 )
extra_biases= [extra_biases motor_bias];

dxl_io.set_motor_pos_speed(17, motor_bias, ...
     0, pi/20);
