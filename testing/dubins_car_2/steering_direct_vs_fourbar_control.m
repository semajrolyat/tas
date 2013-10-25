hold on
data = import_commands('car_planned_commands_directsteer.txt');
planned_commands = process_command_data( data );
plot_command_speed(planned_commands, [1.0,0.0,0.0]);
plot_command_angle(planned_commands, [0.0,1.0,0.0]);

data = import_commands('car_actual_commands_directsteer.txt');
actual_commands = process_command_data( data );
plot_command_speed(actual_commands, [0.5,0.0,1.0]);
plot_command_angle(actual_commands, [0.0,0.8,0.8]);

data = import_commands('car_actual_commands_4barsteer.txt');
actual_commands = process_command_data( data );
plot_command_speed(actual_commands, [0.8,0.6,0.0]);
plot_command_angle(actual_commands, [0.0,0.4,0.8]);

hold off
