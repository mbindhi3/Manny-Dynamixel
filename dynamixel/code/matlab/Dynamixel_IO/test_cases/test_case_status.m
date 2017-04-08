%% Pre-conditions
% Cycle power on OpenCM9.04 controller

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );

%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;
dxl_io.load_library();
dxl_io.connect(5, 1);

motor_ids = 1:22 ;

%% Test Case - Status: Alarm LED
alarm_led_orig = uint8(dxl_io.read_alarm_led(motor_ids, 0));

alarm_led_test = uint8(zeros(size(alarm_led_orig)));
for motor_index = 1:size(alarm_led_orig, 1)
    alarm_led_test(motor_index) = bitcmp(alarm_led_orig(motor_index));
end

dxl_io.set_alarm_led(motor_ids, int32(alarm_led_test));

pause(0.5);

error = uint8(dxl_io.read_alarm_led(motor_ids, 0)) - alarm_led_test;

if ( norm(abs(double(error))) == 0 )
    disp('Test Case - Status: alarm LED passed');
else
    disp('Test Case - Status: alarm LED FAILED');
end

dxl_io.set_alarm_led(motor_ids, 36*ones(length(motor_ids), 1));
pause(0.5);

%% Test Case - Status: Alarm Shutdown
alarm_shutdown_orig = uint8(dxl_io.read_alarm_shutdown(motor_ids, 0));

alarm_shutdown_test = uint8(zeros(size(alarm_shutdown_orig)));
for motor_index = 1:size(alarm_shutdown_orig, 1)
    alarm_shutdown_test(motor_index) = bitcmp(alarm_shutdown_orig(motor_index));
end

dxl_io.set_alarm_shutdown(motor_ids, int32(alarm_shutdown_test));

pause(0.5);

error = uint8(dxl_io.read_alarm_shutdown(motor_ids, 0)) - alarm_shutdown_test;

if ( norm(abs(double(error))) == 0 )
    disp('Test Case - Status: alarm shutdown passed');
else
    disp('Test Case - Status: alarm shutdown FAILED');
end

dxl_io.set_alarm_shutdown(motor_ids, 36*ones(length(motor_ids), 1));
pause(0.5);

%% Test Case - Status: LED
led_orig = dxl_io.read_led(motor_ids, 0);

led_test = zeros(size(led_orig));
for motor_index = 1:size(led_orig, 1)
    led_test(motor_index) = ~(led_orig(motor_index));
end

dxl_io.set_led(motor_ids, led_test);

pause(0.5);

error = dxl_io.read_led(motor_ids, 0) - double(led_test);

if ( norm(abs(error)) == 0 )
    disp('Test Case - Status: LED passed');
else
    disp('Test Case - Status: LED FAILED');
end

dxl_io.set_led(motor_ids, led_orig);
pause(0.5);

%% Test Case - Status: Present Temperature (0 to 85)
present_temp = dxl_io.read_present_temp(motor_ids, 0);

if ( sum(present_temp <= 85) == size(present_temp, 1) )
    disp('Test Case - Status: present temperature passed');
else
    disp('Test Case - Status: present temperature FAILED');
end
pause(0.5);

%% Test Case - Status: Present Voltage
present_volt = dxl_io.read_present_volt(motor_ids, 0);

if ( sum( (present_volt >= 6) + (present_volt <= 19) ) == 2*size(present_volt, 1) )
    disp('Test Case - Status: present voltage passed');
else
    disp('Test Case - Status: present voltage FAILED');
end
pause(0.5);

%% Test Case - Status: Present Load
present_load = dxl_io.read_present_load(motor_ids, 0);

if ( sum( (present_load >= -105) + (present_load <= 105) ) == 2*size(present_load, 1) )
    disp('Test Case - Status: present load passed');
else
    disp('Test Case - Status: present load FAILED');
end
pause(0.5);

%% Test Case - Status: Motor Info
present_motor_info = dxl_io.read_motor_info(14, 1);
error = dxl_io.dxl_retrieve_error(  print_results )
disp('Test Case - Status: Please scan retrieved motor information manually.');
pause(0.5);

