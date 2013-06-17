function [ dydt ] = velocity( t, tfinal, q0, qdes )
%VELOCITY Summary of this function goes here
%   Detailed explanation goes here

dydt = -6 * (qdes - q0) / tfinal^3 * t^2 + 6 * (qdes - q0) / tfinal^2 * t;

end

