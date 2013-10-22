function [ u ] = dubins_IK( q, dq, L )
%dubins_IK computes inverse kinematics for a simple car
%   given the state q = [x,y,theta] in the kinematic frame
%         the change in state dq in the kinematic frame
%         the wheel base of the car L
%         computes the commands u=[speed, steering angle] required 
%         to achieve this change in state

  % get state variables out 
  x = q(1);
  y = q(2);
  theta = q(3);
  
  EPSILON = 1e-5;
  c = cos( theta );      % precompute cos( theta )
  s = sin( theta );      % precompute sin( theta )

  % get dx, dy, dtheta out
  dx = dq(1);
  dy = dq(2);
  dtheta = dq(3);

  % compute speed command u_s = dx/cos(theta) or dx/sin(theta)
  if (abs(c) > EPSILON)
    u_s = dx/c;
  else
    u_s = dy/s;
  end

  % compute the steering command u_phi
  u_phi = atan2( dtheta * L, u_s );  
  
  % return the command vector
  u = [u_s, u_phi ];
end

