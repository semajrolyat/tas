q_initial = [1,0,pi/2];    % initial state
dq_initial = [0,0,0];      % initial velocity
wheel_base = 0.25;         % constant wheel base
steps = 1000;              % number of steps over the path
period = 2*pi;             % length of the path

q_desired = [];            % the desired state
dq_desired = [];           % the desired velocity

% initialization
q = q_initial;             % state vector
dq = dq_initial;           % velocity vector
delta_q = [0,0,0];         % error vector
dt = period / steps;       % step size

% storage for visualization
a_q_desired = [];          % desired state
a_dq_desired = [];         % desired velocity
a_q_actual = [];           % actual state
a_dq_actual = [];          % actual velocity
a_u = [];

K1 = 0.5;                  % constant for scaling error state component
K2 = 0.0001;               % constant for scaling error velocity component
DU = [150 1.357];            % constant for scaling commands

% 2 corrections
%K1 = 0.5;                  % constant for scaling error state component
%K2 = 0.0001;               % constant for scaling error velocity component
%DU = [150 1.5];            % constant for scaling commands

%3 corrections
%K1 = 0.1;                  % constant for scaling error state component
%K2 = 0.0001;               % constant for scaling error velocity component
%DU = [100 1.5];            % constant for scaling commands

% add the initial state into the actual history
a_q_actual = [a_q_actual;q];

resets = 0;

for t = dt:dt:period

    % compute desired state vector
    % compute desired velocity vector

    % get the state prescribed by the function defining the path
    [x_t,y_t,dx_t,dy_t] = lissajous( t, 1, 2, pi/2 );
    
    % compute theta in the world frame
    theta_t = signed_angle(1,0,dx_t,dy_t);
    
    % the state the car should ideally be in at the end of the step
    q_desired = [x_t, y_t, theta_t];
    
    % the velocities that should be required to get to q_desired
    dq_desired = [(x_t - q(1)), (y_t - q(2)), theta_t - q(3)];
    
    % compute the error vector
    delta_q = K1 * (q_desired - q) + K2 * (dq_desired - dq);
    
    % compute the command by inverse kinematics using the current state and
    % the error vector
    u = dubins_IK( q, delta_q, wheel_base ).*DU;
    a_u = [a_u; u];
    
    % compute the ode for the car.
    dq = dubins_FK( q, u, wheel_base );
    qlast = q;
    q = q + dq * dt;
    
    % look for some kind of divergence
    if (norm(q_desired - qlast, inf)*1.01 < norm(q_desired - q, inf))
      q = q_desired;
      resets = resets + 1;
      %disp('Reset necessary\n');
    end

    % update the audits
    a_q_desired = [a_q_desired;q_desired];
    a_dq_desired = [a_dq_desired;dq_desired];
    a_q_actual = [a_q_actual;q];
    a_dq_actual = [a_dq_actual;dq];
end

clf
hold on

disp(sprintf('resets: %d', resets));

plot(a_q_desired(:,1), a_q_desired(:,2),'r');
hold on;
plot(a_q_actual(:,1), a_q_actual(:,2),'b');
hold on;
%plot(size(a_u,1)*dt, a_u(:,2), 'g');

% state plotting
%plot( a_q_desired(:,1), a_q_desired(:,2) );
%for i = 1:n
%  draw_car( a_q_desired(i,:), [1.0,0.0,0.0] );
%  draw_car( a_q_actual(i,:), [0.0,1.0,0.0] );
%end

% position plotting
%plot( a_q_desired(:,1), a_q_desired(:,2), 'LineWidth', 2, 'Color', [1.0,0.0,0.0] );
%plot( a_q_actual(:,1), a_q_actual(:,2), 'LineWidth', 2, 'Color', [0.0,1.0,0.0] );

% velocity plotting
%feather( a_dq_desired(:,1), a_dq_desired(:,2), '-r');
%feather( a_dq_actual(:,1), a_dq_actual(:,2), '-g');

hold off



% A bunch of commented failed attempts at solving this
% noting useful below though

%n = length(q_desired);
%delta = [];
%u = [];

%initialization
%for t = 0:dt:period
%    q_actual = [q_actual;q_initial];
%    dq_actual = [dq_actual;[0,0,0]];
%end

%for i = 1:n-1
    %compute error
%    e = [e;(q_desired(i+1,:) - q_actual(i,:)) + (dq_desired(i+1,:) - dq_actual(i,:))];
    
%    [u_s, u_phi] = dubins_IK(e(i,1), e(i,2), e(i,3), wheel_base);
    
%    s = u_s * dt;
%    r = wheel_base / tan(u_phi);
%    theta = s / r;
    
%    x0 = q_actual(i,1);
%    y0 = q_actual(i,2);
%    theta0 = q_actual(i,3);
    
%    dx = r * cos(theta);
%    dy = r * sin(theta);
%    dtheta = theta;
    
%    x1 = x0 + dx;
%    y1 = y0 + dy;
%    theta1 = theta0 + dtheta;
    
%    q_actual(i+1,:) = [x1,y1,theta1];
%end

%for i = 1:n-1
    %compute error
%    delta = [delta;(q_desired(i+1,:) - q) + (dq_desired(i+1,:) - dq)];
    
%    [u_s, u_phi] = dubins_IK( q(3), delta(i,1), delta(i,2), delta(i,3), wheel_base );
%    u = [u;[u_s,u_phi]];
    
%    dq = dubins_FK( q(3), u(1), u(2), wheel_base );
    
%    [tt,y] = ode45(@dubins_FK, t, y0);
    
%    q = q + dq * dt;
   
%    q_actual(i+1,:) = q;
%    dq_actual(i+1,:) = dq;
%end




