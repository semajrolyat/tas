function [ x,y,dx,dy ] = lissajous( t, a, b, delta )

x = sin(a*t + delta);
y = sin(b*t);

dx = cos(a*t + delta)*a;
dy = cos(b*t)*b;

end

