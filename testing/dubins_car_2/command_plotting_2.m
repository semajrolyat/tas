n = length(v_desired);

hold on
plot(time,phi_actual, 'LineWidth', 2, 'Color', [0.5,0.0,0.0]);
plot(time,phi_desired, 'LineWidth', 2, 'Color', [1.0,0.0,0.0]);

plot(time,v_actual, 'LineWidth', 2, 'Color', [0.5,0.0,0.5]);
plot(time,v_desired, 'LineWidth', 2, 'Color', [1.0,0.0,1.0]);
hold off
