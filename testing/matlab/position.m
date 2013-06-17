function [ y ] = position( t, tfinal, q0, qdes )
%POSITION Summary of this function goes here
%   Detailed explanation goes here

y = -2 * (qdes - q0) / tfinal^3 * t^3 + 3 * (qdes - q0) / tfinal^2 * t^2 + q0;

end

