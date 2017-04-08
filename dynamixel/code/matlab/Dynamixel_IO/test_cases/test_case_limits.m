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

%% Test Case - Limits: Torque Limit
torque_limit_orig = dxl_io.read_torque_limit(motor_ids, 0);

torque_limit_test = zeros(size(torque_limit_orig));
for motor_index = 1:size(torque_limit_orig, 1)
    torque_limit_test(motor_index) = torque_limit_orig(motor_index) - motor_index;
end

dxl_io.set_torque_limit(motor_ids, torque_limit_test);

pause(0.5);

error = dxl_io.read_torque_limit(motor_ids, 0) - torque_limit_test;

if ( norm(abs(error)) <= 0.1*size(error, 1) )
    disp('Test Case - Limits: torque limit passed');
else
    disp('Test Case - Limits: torque limit FAILED');
end

dxl_io.set_torque_limit(motor_ids, torque_limit_orig);
pause(0.5);

%% Test Case - Limits: Max. Torque
max_torque_orig = dxl_io.read_max_torque(motor_ids, 0);

max_torque_test = zeros(size(max_torque_orig));
for motor_index = 1:size(max_torque_orig, 1)
    max_torque_test(motor_index) = max_torque_orig(motor_index) - motor_index;
end

dxl_io.set_max_torque(motor_ids, max_torque_test);

pause(0.5);

error = dxl_io.read_max_torque(motor_ids, 0) - max_torque_test;

if ( norm(abs(error)) <= 0.1*size(error, 1) )
    disp('Test Case - Limits: max torque passed');
else
    disp('Test Case - Limits: max torque FAILED');
end

dxl_io.set_max_torque(motor_ids, 100*ones(length(motor_ids), 1));
pause(0.5);

%% Test Case - Limits: Angle Limit
angle_limit_orig = dxl_io.read_angle_limit(motor_ids, 0);

angle_limit_test = zeros(size(angle_limit_orig));
angle_limit_test(:, 1) = angle_limit_orig(:, 1) + 10*(pi/180);
angle_limit_test(:, 2) = angle_limit_orig(:, 2) - 10*(pi/180);

dxl_io.set_angle_limit(motor_ids, angle_limit_test(:, 1), 'cw');
dxl_io.set_angle_limit(motor_ids, angle_limit_test(:, 2), 'ccw');

pause(0.5);

error_angle_lim_cw = dxl_io.read_angle_limit(motor_ids, 0, 'cw') - angle_limit_test(:, 1);
error_angle_lim_ccw = dxl_io.read_angle_limit(motor_ids, 0, 'ccw')- angle_limit_test(:, 2);

if ( (norm(error_angle_lim_cw) <= 0.3 *size(error, 1)) && (norm(error_angle_lim_ccw) <= 0.3 *size(error, 1)) )
    disp('Test Case - Limits: angle limit passed');
else
    disp('Test Case - Limits: angle limit FAILED');
end

dxl_io.set_angle_limit(motor_ids, [ -150*(pi/180)*ones(length(motor_ids), 1) dxl_io.pos_cnt_to_rad(1023)*ones(length(motor_ids), 1) ] );
pause(0.5);

%% Test Case - Limits: Voltage Limit
voltage_limit_orig = dxl_io.read_voltage_limit(motor_ids, 0);

voltage_limit_test = zeros(size(voltage_limit_orig));
voltage_limit_test(:, 1) = voltage_limit_orig(:, 1) + 1;
voltage_limit_test(:, 2) = voltage_limit_orig(:, 2) - 1;

dxl_io.set_voltage_limit(motor_ids, voltage_limit_test(:, 1), 'low');
dxl_io.set_voltage_limit(motor_ids, voltage_limit_test(:, 2), 'high');

pause(0.5);

error_volt_lim_low = dxl_io.read_voltage_limit(motor_ids, 0, 'low') - voltage_limit_test(:, 1);
error_volt_lim_high = dxl_io.read_voltage_limit(motor_ids, 0, 'high')- voltage_limit_test(:, 2);

if ( (norm(error_volt_lim_low) <= 0.1 *size(error, 1)) && (norm(error_volt_lim_high) <= 0.1 *size(error, 1)) )
    disp('Test Case - Limits: voltage limit passed');
else
    disp('Test Case - Limits: voltage limit FAILED');
end

dxl_io.set_voltage_limit(motor_ids, [6*ones(length(motor_ids), 1) 19*ones(length(motor_ids), 1) ]);
pause(0.5);

%% Test Case - Limits: Temp. Limit
temp_limit_orig = dxl_io.read_temp_limit(motor_ids, 0);

temp_limit_test = zeros(size(temp_limit_orig));
for motor_index = 1:size(temp_limit_orig, 1)
    temp_limit_test(motor_index) = temp_limit_orig(motor_index) + motor_index;
end

dxl_io.set_temp_limit(motor_ids, temp_limit_test);

pause(0.5);

error = dxl_io.read_temp_limit(motor_ids, 0) - temp_limit_test;

if ( norm(abs(error)) == 0 )
    disp('Test Case - Limits: temperature limit passed');
else
    disp('Test Case - Limits: temperature limit FAILED');
end

dxl_io.set_temp_limit(motor_ids, 80*ones(length(motor_ids), 1));
pause(0.5);
