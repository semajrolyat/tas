function [ phi ] = kingpin_angle( steering_angle, STEERING_LEVER_BASE_ANGLE,...
                                  BASE_LINK_LENGTH, STEERING_LEVER_LENGTH,...
                                  TIEROD_LENGTH, SPINDLE_LEVER_LENGTH,...
                                  SPINDLE_LEVER_BASE_ANGLE )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
  BASE_LINK_LENGTH_SQ = BASE_LINK_LENGTH * BASE_LINK_LENGTH;
  STEERING_LEVER_LENGTH_SQ = STEERING_LEVER_LENGTH * STEERING_LEVER_LENGTH;
  TIEROD_LENGTH_SQ = TIEROD_LENGTH * TIEROD_LENGTH;
  SPINDLE_LEVER_LENGTH_SQ = SPINDLE_LEVER_LENGTH * SPINDLE_LEVER_LENGTH;

% compute the kingpin angle
  theta = STEERING_LEVER_BASE_ANGLE - steering_angle;
  h_sq = BASE_LINK_LENGTH_SQ + STEERING_LEVER_LENGTH_SQ - ...
         2.0 * BASE_LINK_LENGTH * STEERING_LEVER_LENGTH * cos( theta );
  h = sqrt( h_sq );

  rho = acos( (BASE_LINK_LENGTH_SQ + h_sq - STEERING_LEVER_LENGTH_SQ) /...
        (2.0 * BASE_LINK_LENGTH * h) ) +...
        acos( (SPINDLE_LEVER_LENGTH_SQ + h_sq - TIEROD_LENGTH_SQ) /...
        (2.0 * SPINDLE_LEVER_LENGTH * h) );
  phi = rho - (pi - SPINDLE_LEVER_BASE_ANGLE);
end

