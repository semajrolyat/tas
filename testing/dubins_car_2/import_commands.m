function [ commands ] = import_commands( file )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here

  data = importdata(file);

  commands = data.data;

end

