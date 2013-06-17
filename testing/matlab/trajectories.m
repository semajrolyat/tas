
tfinal = 1;
p = [];
v = [];

%q0 = 0;
%qdes = pi;

q0 = -pi / 2;
qdes = pi / 2;


interval = 0:0.01:tfinal;

for t = interval

p_t = position( t, tfinal, q0, qdes );
v_t = velocity( t, tfinal, q0, qdes );
p = [p, p_t];
v = [v, v_t];

end

figure
plot( interval, p, '-r' );
hold on
plot( interval, v, '-b' );