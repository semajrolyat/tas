function [ A ] = process_command_data( data )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

  n = length(data(:,1));

  STEP = 0.001;

  tf = 0;
  for i = 1:n
    tf = tf + data(i,1);
  end

  STEPS = int64(tf / STEP);
  A = zeros(STEPS,4);

  t = 0;
  j = 1;
  d = data(j,1);
  u_speed = data(j,2);
  u_phi = data(j,3);
  
  for i = 1:STEPS
    A(i,1) = t;
    A(i,2) = d;
    A(i,3) = u_speed;
    A(i,4) = u_phi;
    d = d - STEP;
    if( d <= 0 )
      j = j + 1;
      if( j > n )
          return;
      end
      d = data(j,1);
      u_speed = data(j,2);
      u_phi = data(j,3);
    end
    t = t + STEP;
  end
end

