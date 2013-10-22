function draw_car( q, color )
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

u = cos(q(3));
v = sin(q(3));

quiver( q(1), q(2), u, v, 0.1,'LineWidth',2, 'Color', color );
plot( q(1),q(2),'-s','LineWidth',2,'MarkerSize',20, 'Color', color);

end

