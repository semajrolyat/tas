hold on;

import_statefile('car_planned_commands.txt')
n = length(speed);

t1 = 0;
time1 = [];
for i = 1:n
    time1 = [time1,t1];
    t1 = t1 + duration(i);
end


plot(time1,angle, 'LineWidth', 2, 'Color', [0.0,1.0,0.0]);

plot(time1,speed, 'LineWidth', 2, 'Color', [0.0,0.0,0.5]);


import_statefile('car_actual_commands.txt')
n2 = length(speed);

t2 = 0;
time2 = [];
for i = 1:n2
    time2 = [time2,t2];
    t2 = t2 + duration(i);
end


plot(time2,angle, 'LineWidth', 2, 'Color', [0.0,0.5,0.0]);

plot(time2,speed, 'LineWidth', 2, 'Color', [0.5,0.0,0.0]);



hold off
