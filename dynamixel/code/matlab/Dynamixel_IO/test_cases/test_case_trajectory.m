%% Pre-conditions
% Cycle power on OpenCM9.04 controller

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );

%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;
dxl_io.load_library();
dxl_io.connect(0, 1);

motor_ids = 0:10;

%% Test Case - Trajectory: Goal Position/Speed
dest_pos_base = pi/6;
dest_speed_base = dest_pos_base/2;

dest_pos = zeros(length(motor_ids), 1);
dest_speed = zeros(length(motor_ids), 1);
for motor_index = 1:length(motor_ids)
   dest_pos(motor_index) = dest_pos_base + (motor_index)*(pi/180);
   dest_speed(motor_index) = dest_speed_base;
end

input('Orient Snakey with smooth side against the ground, in a straight-line configuration. Press <Enter> to begin goal position/speed test.\n');
dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), dest_pos, dest_speed );

pause(0.75);
moving = dxl_io.is_moving(motor_ids, 0);
vel_trans = dxl_io.read_present_pos_vel( motor_ids, 0, 'vel');

pause(2);
pos_vel_final = dxl_io.read_present_pos_vel( motor_ids, 0);

goal_pos_speed = dxl_io.read_goal_pos_speed( motor_ids, 0);

if ( sum(moving) == length(motor_ids) )
    disp('Test Case - Trajectory: is moving passed');
else
    disp('Test Case - Trajectory: is moving FAILED');
end

if ( sum(abs(vel_trans) > 0) == length(motor_ids) )
    disp('Test Case - Trajectory: read present velocity passed');
else
    disp('Test Case - Trajectory: read present velocity FAILED');
end

if ( norm(abs(pos_vel_final(:, 1) - dest_pos)) <= length(motor_ids)*(3*pi/180) )
    disp('Test Case - Trajectory: read present position passed');
else
    disp('Test Case - Trajectory: read present position FAILED');
end

error_goal_pos = goal_pos_speed - [dest_pos dest_speed];

if ( norm(abs(error_goal_pos)) <= length(motor_ids)*2*(3*pi/180) )
    disp('Test Case - Trajectory: read goal position/speed passed');
else
    disp('Test Case - Trajectory: read goal position/speed FAILED');
end
pause(1);

%% Test Case - Trajectory: Goal Position/Speed
dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), zeros(length(motor_ids), 1), dest_speed );

pause(2);
dxl_io.set_torque_enable( motor_ids, zeros(length(motor_ids), 1));
pass_fail = input('Enter p if torque has been disabled on all motors. Enter f otherwise. Submit response with <Enter>.\n');

dxl_io.set_torque_enable( motor_ids, ones(length(motor_ids), 1));
pres_torque_enable = dxl_io.read_torque_enable( motor_ids, 0 );

error_torque_enable = pres_torque_enable - ones(length(motor_ids), 1);
if ( strcmp(pass_fail, 'p') && (sum(error_torque_enable) == 0) )
    disp('Test Case - Trajectory: torque enable passed');
else
    disp('Test Case - Trajectory: torque enable FAILED');
end
pause(1);

%% Test Case - Trajectory: Execute Trajectory
load('test_case_lat_undulation_gait.mat');

dxl_io.set_torque_enable( motor_ids, zeros(length(motor_ids), 1));
pause(0.5);
joint_bias = int32(dxl_io.calibrate_motor(0:10, 1));
pause(0.5);
dxl_io.set_torque_enable( motor_ids, ones(length(motor_ids), 1));
pause(0.5);

if ( sum(abs(joint_bias) < 100) == length(motor_ids) )
    disp('Test Case - Trajectory: calibrate motor passed');
else
    disp('Test Case - Trajectory: calibrate motor FAILED');
end

dxl_io.set_compliance_slope( 1:2:10, 8*ones(5, 2) );
input('Press <Enter> to begin trajectory execution.\n');
dxl_io.execute_trajectory( 0, 0:10, joint_bias, theta(:, 1), vel(:, 1), time(:, 1), joint_compl_margin(:, 1) );
pause(1);
dxl_io.execute_trajectory( 0, 0:10, joint_bias, theta, vel, time, joint_compl_margin );

pass_fail = input('Enter p if lateral undulation gait executed. Enter f otherwise. Submit response with <Enter>.\n');
if ( strcmp(pass_fail, 'p') )
    disp('Test Case - Trajectory: execute trajectory passed');
else
    disp('Test Case - Trajectory: execute trajectory FAILED');
end
pause(1);

