import_statefile('car_planned_commands.txt')
n = length(speed);

t = 0;
time = [];
for i = 1:n
    time = [time,t];
    t = t + duration(i);
end

hold on
plot(time,angle, 'LineWidth', 2, 'Color', [0.0,0.5,0.0]);

%plot(time,speed, 'LineWidth', 2, 'Color', [0.0,0.0,0.5]);

hold off
