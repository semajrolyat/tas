% constants
%SPINDLE_LENGTH = 0.00;
BASE_LINK_LENGTH = 0.12;
MAX_STEERING_ANGLE_DEG = 30.0;
MAX_STEERING_ANGLE = MAX_STEERING_ANGLE_DEG * pi / 180.0;
EPSILON = 1e-6;

MAX_ITS = 1e10;
it = 0;

steering_angle = MAX_STEERING_ANGLE;

STEERING_LEVER_BASE_ANGLE_DEG = 75.0;
STEERING_LEVER_BASE_ANGLE = STEERING_LEVER_BASE_ANGLE_DEG * pi / 180.0;

STEERING_LEVER_LENGTH = 0.0562;
TIEROD_LENGTH = 0.1;
SPINDLE_LEVER_LENGTH = 0.055;

quit = false;
solved = false;

while(~quit)
    
[left, right] = double_four_bar( steering_angle, STEERING_LEVER_BASE_ANGLE,...
                                 BASE_LINK_LENGTH, STEERING_LEVER_LENGTH,...
                                 TIEROD_LENGTH, SPINDLE_LEVER_LENGTH);
                             
avg = (left + right) / 2.0;
it = it + 1;
if( it == MAX_ITS )
    quit = true;
end

if( abs (steering_angle - avg) < EPSILON )
    solved = true;
    quit = true;
end

STEERING_LEVER_LENGTH = STEERING_LEVER_LENGTH - 1e-7;

end

disp(sprintf('[ solved:%d, steering_lever_l:%f, tie_rod_l:%f, spindle_lever_l:%f ]', solved, STEERING_LEVER_LENGTH, TIEROD_LENGTH, SPINDLE_LEVER_LENGTH));