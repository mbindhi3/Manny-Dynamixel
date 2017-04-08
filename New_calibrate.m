%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );

%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(5, 1);   % connect to port 0, at 1 MBaud

motor_ids = [12,14,16, 11,13,15];      % list of motor IDs to command
Motor_Biases=zeros(1,22);
% extra_biases=[];
%% Test Case - Trajectory: Goal Position/Speed
motor_ids_initial =[ 1:5,10, 17:22];
dxl_io.set_torque_enable(motor_ids_initial,zeros(1,12)');
dxl_io.set_torque_enable(motor_ids,zeros(1,6)');

motor_bias = dxl_io.calibrate_motor(10, 1 )
Motor_Biases(10)= motor_bias;

motor_bias = dxl_io.calibrate_motor(20, 1 )
Motor_Biases(20)= motor_bias;

motor_bias = dxl_io.calibrate_motor(12, 1 )
Motor_Biases(12)= motor_bias;

motor_bias = dxl_io.calibrate_motor(14, 1 )
Motor_Biases(14)= motor_bias;

motor_bias = dxl_io.calibrate_motor(16, 1 )
Motor_Biases(16)= motor_bias;

motor_bias = dxl_io.calibrate_motor(18, 1 )
Motor_Biases(18)= motor_bias;

motor_bias = dxl_io.calibrate_motor(9, 1 )
Motor_Biases(9)= motor_bias;

motor_bias = dxl_io.calibrate_motor(19, 1 )
Motor_Biases(19)= motor_bias;
  
motor_bias = dxl_io.calibrate_motor(11, 1 )
Motor_Biases(11)= motor_bias; 

motor_bias = dxl_io.calibrate_motor(13, 1 )
Motor_Biases(13)= motor_bias;
 
motor_bias = dxl_io.calibrate_motor(15, 1 )
Motor_Biases(15)= motor_bias;
 
motor_bias = dxl_io.calibrate_motor(17, 1 )
Motor_Biases(17)= motor_bias;

