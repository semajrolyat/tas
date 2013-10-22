function [ theta ] = signed_angle( ux, uy, vx, vy )

    u = [ux,uy,0.0];
    v = [vx,vy,0.0];
    u = u/norm(u);
    v = v/norm(v);
    
    theta = atan2(u(1)*v(2)-u(2)*v(1), dot(u,v));
end

