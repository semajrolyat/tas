import_commandfile('predator_commandset.txt')
n = length(duration);

time = [];
t1 = 0;
for i = 1:n
    t1 = t1 + duration(i);
    time = [time,t1];
end

hold on
plot(time,phi, 'LineWidth', 2, 'Color', [0.0,0.0,1.0]);
plot(time,speed, 'LineWidth', 2, 'Color', [1.0,0.0,0.0]);
hold off
