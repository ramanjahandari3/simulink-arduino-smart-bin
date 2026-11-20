%% build_lab1_models.m
% Generates the LAB1 Simulink deliverables:
%   dc_motor.slx, dc_constants.sldd, pid_controller.slx, pid_constants.sldd
%
% Put this script in the same folder as top_level.slx, then run it in MATLAB.

clear; clc;

%% Clean previously generated files if they exist
files_to_remove = {'dc_motor.slx','dc_constants.sldd', ...
                   'pid_controller.slx','pid_constants.sldd'};
for k = 1:numel(files_to_remove)
    if exist(files_to_remove{k}, 'file')
        delete(files_to_remove{k});
    end
end

%% ------------------------------------------------------------------------
% 1) Create data dictionary for DC motor
% -------------------------------------------------------------------------
dc_dd = Simulink.data.dictionary.create('dc_constants.sldd');
dc_sec = getSection(dc_dd, 'Design Data');

addEntry(dc_sec, 'J',  0.01);  % kg*m^2
addEntry(dc_sec, 'b',  0.1);   % N*m*s
addEntry(dc_sec, 'Ke', 0.01);  % V/(rad/s)
addEntry(dc_sec, 'Kt', 0.01);  % N*m/A
addEntry(dc_sec, 'R',  1);     % Ohm
addEntry(dc_sec, 'L',  0.5);   % H

saveChanges(dc_dd);
close(dc_dd);

%% ------------------------------------------------------------------------
% 2) Create DC motor model
% Equations:
%   d(omega)/dt = (Kt*i - b*omega)/J
%   d(i)/dt     = (-R*i + V - Ke*omega)/L
% Output:
%   omega = dtheta/dt
% -------------------------------------------------------------------------
mdl = 'dc_motor';
new_system(mdl);
open_system(mdl);

set_param(mdl, 'DataDictionary', 'dc_constants.sldd');
set_param(mdl, 'StopTime', '10');

% Blocks
add_block('simulink/Sources/In1',           blk(mdl,'V_V'),              'Position',[30 100 60 120]);
add_block('simulink/Math Operations/Sum',   blk(mdl,'sum_electrical'),   'Position',[150 80 175 140], 'Inputs','|+--');
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_inv_L'),       'Position',[220 95 270 125], 'Gain','1/L');
add_block('simulink/Continuous/Integrator', blk(mdl,'integrator_i'),     'Position',[315 95 345 125]);
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_R'),           'Position',[400 35 450 65],  'Gain','R');
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_Kt'),          'Position',[400 115 450 145], 'Gain','Kt');

add_block('simulink/Math Operations/Sum',   blk(mdl,'sum_mechanical'),   'Position',[530 115 555 165], 'Inputs','|+-');
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_inv_J'),       'Position',[600 130 650 160], 'Gain','1/J');
add_block('simulink/Continuous/Integrator', blk(mdl,'integrator_omega'), 'Position',[700 130 730 160]);
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_b'),           'Position',[785 185 835 215], 'Gain','b');
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_Ke'),          'Position',[785 75 835 105],  'Gain','Ke');
add_block('simulink/Sinks/Out1',            blk(mdl,'speed_rad/s'),      'Position',[900 135 930 155]);

% Lines
add_line(mdl, 'V_V/1',              'sum_electrical/1',   'autorouting','on');
add_line(mdl, 'sum_electrical/1',   'gain_inv_L/1',       'autorouting','on');
add_line(mdl, 'gain_inv_L/1',       'integrator_i/1',     'autorouting','on');
add_line(mdl, 'integrator_i/1',     'gain_R/1',           'autorouting','on');
add_line(mdl, 'gain_R/1',           'sum_electrical/2',   'autorouting','on');
add_line(mdl, 'integrator_i/1',     'gain_Kt/1',          'autorouting','on');
add_line(mdl, 'gain_Kt/1',          'sum_mechanical/1',   'autorouting','on');

add_line(mdl, 'sum_mechanical/1',   'gain_inv_J/1',       'autorouting','on');
add_line(mdl, 'gain_inv_J/1',       'integrator_omega/1', 'autorouting','on');
add_line(mdl, 'integrator_omega/1', 'speed_rad//s/1',     'autorouting','on');
add_line(mdl, 'integrator_omega/1', 'gain_b/1',           'autorouting','on');
add_line(mdl, 'gain_b/1',           'sum_mechanical/2',   'autorouting','on');
add_line(mdl, 'integrator_omega/1', 'gain_Ke/1',          'autorouting','on');
add_line(mdl, 'gain_Ke/1',          'sum_electrical/3',   'autorouting','on');

save_system(mdl, 'dc_motor.slx');
close_system(mdl);

%% ------------------------------------------------------------------------
% 3) Create data dictionary for PID controller
% -------------------------------------------------------------------------
pid_dd = Simulink.data.dictionary.create('pid_constants.sldd');
pid_sec = getSection(pid_dd, 'Design Data');

addEntry(pid_sec, 'Kp', 100);
addEntry(pid_sec, 'Ki', 200);
addEntry(pid_sec, 'Kd', 10);

saveChanges(pid_dd);
close(pid_dd);

%% ------------------------------------------------------------------------
% 4) Create PID controller model
% Equation:
%   cmd_V = Kp*error + Ki*integral(error) + Kd*derivative(error)
% -------------------------------------------------------------------------
mdl = 'pid_controller';
new_system(mdl);
open_system(mdl);

set_param(mdl, 'DataDictionary', 'pid_constants.sldd');
set_param(mdl, 'StopTime', '10');

% Blocks
add_block('simulink/Sources/In1',           blk(mdl,'error_rad/s'),    'Position',[30 120 60 140]);
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_Kp'),        'Position',[160 55 220 85],   'Gain','Kp');
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_Ki'),        'Position',[160 120 220 150], 'Gain','Ki');
add_block('simulink/Continuous/Integrator', blk(mdl,'integrator_error'),'Position',[280 120 310 150]);
add_block('simulink/Continuous/Derivative', blk(mdl,'derivative_error'),'Position',[160 190 220 220]);
add_block('simulink/Math Operations/Gain',  blk(mdl,'gain_Kd'),        'Position',[280 190 340 220], 'Gain','Kd');
add_block('simulink/Math Operations/Sum',   blk(mdl,'sum_pid_terms'),  'Position',[430 115 455 175], 'Inputs','|+++');
add_block('simulink/Sinks/Out1',            blk(mdl,'cmd_V'),          'Position',[540 135 570 155]);

% Lines
add_line(mdl, 'error_rad//s/1',     'gain_Kp/1',         'autorouting','on');
add_line(mdl, 'error_rad//s/1',     'gain_Ki/1',         'autorouting','on');
add_line(mdl, 'gain_Ki/1',          'integrator_error/1','autorouting','on');
add_line(mdl, 'error_rad//s/1',     'derivative_error/1','autorouting','on');
add_line(mdl, 'derivative_error/1', 'gain_Kd/1',         'autorouting','on');

add_line(mdl, 'gain_Kp/1',          'sum_pid_terms/1',   'autorouting','on');
add_line(mdl, 'integrator_error/1', 'sum_pid_terms/2',   'autorouting','on');
add_line(mdl, 'gain_Kd/1',          'sum_pid_terms/3',   'autorouting','on');
add_line(mdl, 'sum_pid_terms/1',    'cmd_V/1',           'autorouting','on');

save_system(mdl, 'pid_controller.slx');
close_system(mdl);

%% ------------------------------------------------------------------------
% 5) Basic check with top_level.slx
% -------------------------------------------------------------------------
if exist('top_level.slx','file')
    open_system('top_level.slx');
    set_param('top_level', 'StopTime', '10');
    sim('top_level');
    disp('Simulation completed successfully.');
else
    warning('top_level.slx was not found in the current folder. Copy it here and run the simulation manually.');
end

%% Local helper: escape "/" inside Simulink block names
function path = blk(model_name, block_name)
    path = [model_name '/' strrep(block_name, '/', '//')];
end
