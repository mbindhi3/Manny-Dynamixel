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

%% Test Case - Compliance: Compliance Margin
compl_margin_orig = int32(dxl_io.read_compliance_margin(motor_ids, 0)*180/pi*1024/300);

compl_margin_test = zeros(size(compl_margin_orig));
compl_margin_test(:, 1) = compl_margin_orig(:, 1) + 2;
compl_margin_test(:, 2) = compl_margin_orig(:, 2) + 4;

dxl_io.set_compliance_margin(motor_ids, compl_margin_test(:, 1), 'cw');
dxl_io.set_compliance_margin(motor_ids, compl_margin_test(:, 2), 'ccw');

pause(0.5);

error_compl_marg_cw = dxl_io.read_compliance_margin(motor_ids, 0, 'cw')*180/pi*1024/300 - compl_margin_test(:, 1);
error_compl_marg_ccw = dxl_io.read_compliance_margin(motor_ids, 0, 'ccw')*180/pi*1024/300- compl_margin_test(:, 2);

if ( (norm(error_compl_marg_cw) + norm(error_compl_marg_ccw))  < 0.1 )
    disp('Test Case - Compliance: compliance margin passed');
else
    disp('Test Case - Compliance: compliance margin FAILED');
end

dxl_io.set_compliance_margin(motor_ids, compl_margin_orig);
pause(0.5);

%% Test Case - Compliance: Compliance Slope
compl_slope_orig = dxl_io.read_compliance_slope(motor_ids, 0);

compl_slope_test = zeros(size(compl_slope_orig));
for motor_index = 1:size(compl_slope_orig, 1)
    compl_slope_test(motor_index, 1) = bitshift(compl_slope_orig(motor_index, 1), -1);
    compl_slope_test(motor_index, 2) = bitshift(compl_slope_orig(motor_index, 2), 1);
end

dxl_io.set_compliance_slope(motor_ids, compl_slope_test(:, 1), 'cw');
dxl_io.set_compliance_slope(motor_ids, compl_slope_test(:, 2), 'ccw');

pause(0.5);

error_compl_slope_cw = dxl_io.read_compliance_slope(motor_ids, 0, 'cw') - compl_slope_test(:, 1);
error_compl_slope_ccw = dxl_io.read_compliance_slope(motor_ids, 0, 'ccw')- compl_slope_test(:, 2);

if ( (norm(abs(error_compl_slope_cw)) + norm(abs(error_compl_slope_ccw)))  == 0 )
    disp('Test Case - Compliance: compliance slope passed');
else
    disp('Test Case - Compliance: compliance slope FAILED');
end

dxl_io.set_compliance_slope(motor_ids, compl_slope_orig);
pause(0.5);

%% Test Case - Compliance: Punch
punch_orig = dxl_io.read_punch(motor_ids, 0)*10;

punch_test = zeros(size(punch_orig));
for motor_index = 1:size(compl_slope_orig, 1)
    punch_test(motor_index) = punch_orig(motor_index) + motor_index;
end

dxl_io.set_punch(motor_ids, punch_test);

pause(0.5);

error = dxl_io.read_punch(motor_ids, 0)*10 - punch_test;

if ( norm(abs(error)) == 0 )
    disp('Test Case - Compliance: punch passed');
else
    disp('Test Case - Compliance: punch FAILED');
end

dxl_io.set_punch(motor_ids, punch_orig);
pause(0.5);
