hold on

import_statefile('car_planned_states_directsteer.txt')
n = length(x);
for i=1:n
    while theta(i) < 0
        theta(i) = theta(i) + 2*pi;
    end
    while theta(i) >= 2*pi
        theta(i) = theta(i) - 2*pi;
    end   
end

u = cos(theta);
v = sin(theta);

plot(x,y,'LineWidth', 2, 'Color', [1.0,0.0,0.0]);
%quiver(x,y,u,v,0.1,'LineWidth',2, 'Color', [1.0,0.0,1.0]);
plot(x(1),y(1),'-ko','LineWidth',2,'MarkerSize',10);
plot(x(n),y(n),'-kx','LineWidth',2,'MarkerSize',10);



import_statefile('car_actual_states_directsteer.txt')
n = length(x);
for i=1:n
    while theta(i) < 0
        theta(i) = theta(i) + 2*pi;
    end
    while theta(i) >= 2*pi
        theta(i) = theta(i) - 2*pi;
    end   
end

u = cos(theta);
v = sin(theta);

plot(x,y,'LineWidth', 2, 'Color', [0.0,1.0,0.0]);
%quiver(x,y,u,v,0.1,'LineWidth',2, 'Color', [0.0,0.0,1.0]);
plot(x(1),y(1),'-ko','LineWidth',2,'MarkerSize',10);
plot(x(n),y(n),'-kx','LineWidth',2,'MarkerSize',10);

import_statefile('car_actual_states_4barsteer.txt')
n = length(x);
for i=1:n
    while theta(i) < 0
        theta(i) = theta(i) + 2*pi;
    end
    while theta(i) >= 2*pi
        theta(i) = theta(i) - 2*pi;
    end   
end

u = cos(theta);
v = sin(theta);

plot(x,y,'LineWidth', 2, 'Color', [0.0,0.0,1.0]);
%quiver(x,y,u,v,0.1,'LineWidth',2, 'Color', [0.0,0.0,1.0]);
plot(x(1),y(1),'-ko','LineWidth',2,'MarkerSize',10);
plot(x(n),y(n),'-kx','LineWidth',2,'MarkerSize',10);

hold off