%Dynamixel control

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );
clear all
%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(5, 1);   % connect to port 0, at 1 MBaud

alphaTraj = 'alphas.mat';
motor_ids = [12,14,16, 11,13,15];      % list of motor IDs to command
load(alphaTraj);
a =alphas;
start =1;

load('Motor_Biases.mat');
Motor_Biases = -1* motor_bias;
final = 600;

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
pause(2);
motorBias =[Motor_Biases(4:2:8); Motor_Biases(3:2:7)];
%motor_bias =[-1;4;-2;4;-2;2];
dxl_io.set_motor_pos_speed(motor_ids,motorBias, ...
   [a(1,start); -a(2,start); a(3,start);-a(4,start); a(5,start); -a(6,start)], ones(length(motor_ids),1)*pi/6);

downSample = 1;
timeStep = 0.1;
pauseTime = .13;
speed = [];
input('Press <Enter> to begin goal position/speed test.\n');
while(1)
WalkTrial(start,final);
end


% for i = start + 2:downSample:final;
% index = i
% dest_pos_base(1) = a(1,i) ;% destination position
% dest_pos_base(2) = -a(2,i) ;% destination position
% dest_pos_base(3) = a(3,i) ;% destination position
% dest_pos_base(4) = -a(4,i) ;% destination position
% dest_pos_base(5) = a(5,i) ;% destination position
% dest_pos_base(6) = -a(6,i) ;% destination position
% 
% dest_speed_base(1) = max(abs((a(1,i) - a(1,i-downSample)) /timeStep),3 *0.1745);    % speed
% dest_speed_base(2) = max(abs((a(2,i) - a(2,i-downSample)) /timeStep),3 * 0.1745);    % speed
% dest_speed_base(3) = max(abs((a(3,i) - a(3,i-downSample)) /timeStep),3 * 0.1745);    % speed
% dest_speed_base(4) = max(abs((a(4,i) - a(4,i-downSample)) /timeStep),3 * 0.1745);    % speed
% dest_speed_base(5) = max(abs((a(5,i) - a(5,i-downSample)) /timeStep),3 * 0.1745);    % speed
% dest_speed_base(6) = max(abs((a(6,i) - a(6,i-downSample)) /timeStep),3 * 0.1745);    % speed
% 
% dest_pos = zeros(length(motor_ids), 1);         % array destination positions to assign for each motor ID
% dest_speed = zeros(length(motor_ids), 1);       % array speeds to assign for each motor ID
% for motor_index = 1:length(motor_ids)
%    dest_pos(motor_index) = dest_pos_base(motor_index);
%    dest_speed(motor_index) = dest_speed_base(motor_index);
% end
% if(i==start)
% input('Press <Enter> to begin goal position/speed test.\n');
% dest_speed_base(1) = dest_speed_base(1)/50;  
% dest_speed_base(2) = dest_speed_base(2)/50;    
% dest_speed_base(3) = dest_speed_base(3)/50;   
% dest_speed_base(4) = dest_speed_base(4)/50;   
% dest_speed_base(5) = dest_speed_base(5)/50;    
% dest_speed_base(6) = dest_speed_base(6)/50;
% dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), dest_pos, dest_speed);
% pause(pauseTime);
% else
% dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1),dest_pos, dest_speed);
% pause(pauseTime);
% end
% speed = [speed ; dest_speed_base];
% end 
%end