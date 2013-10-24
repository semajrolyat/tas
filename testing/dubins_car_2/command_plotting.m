import_statefile('car_planned_commands.txt')
n = length(speed);

STEP = 0.001;

t0 = 0;
tf = 0;
time = [];

for i = 1:n
  tf = tf + duration(i);
end

STEPS = int64(tf / STEP);
A = zeros(STEPS,4);

t = 0;
j = 1;
d = duration(1);
u_speed = speed(1);
u_phi = angle(1);
for i = 1:STEPS
  A(i,1) = t;
  A(i,2) = d;
  A(i,3) = u_speed;
  A(i,4) = u_phi;
  d = d - STEP;
  if( d < 0 )
      j = j + 1;
      d = duration(j);
      u_speed = speed(j);
      u_phi = angle(j);
  end
  t = t + STEP;
end

hold on
plot(A(:,1),A(:,4), 'LineWidth', 2, 'Color', [0.0,0.5,0.0]);

plot(A(:,1),A(:,3), 'LineWidth', 2, 'Color', [0.0,0.0,0.5]);

hold off
