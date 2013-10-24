function plot_command_file( data, color_speed, color_angle  )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

  
  plot( data(:,1), data(:,3), 'LineWidth', 2, 'Color', color_speed );
  plot( data(:,1), data(:,4), 'LineWidth', 2, 'Color', color_angle );
  
end

