function [ dq ] = dubins_FK( q, u, L )
%dubins_FK computes forward kinematics for a simple car
%   given the state q = [x,y,theta] in the kinematic frame
%         the commands u = [s, phi] 
%         the wheel base of the car L
%         computes the change in state dq

  % state
  x = q(1);
  y = q(2);
  theta = q(3);
  
  %commands
  u_s = u(1);
  u_phi = u(2);
  
  %ode
  dx = u_s * cos( theta );
  dy = u_s * sin( theta );
  dtheta = u_s * tan(u_phi)/L;
  
  % return dstate
  dq = [ dx dy dtheta ];
end

