% lqr


tfinal = 1;
p= [];
v=[];

for i = 0:0.01:tfinal

p_i = position( i, tfinal, 0, pi );
v_i = velocity( i, tfinal, 0, pi );
p = [p, p_i];
v = [v, v_i];
%p = position( 0.5, tfinal, 0, pi );
%v = velocity( 0.5, tfinal, 0, pi );

end

%figure
%plot(0:0.01:tfinal, p);

%figure
%plot(0:0.01:tfinal, v);


figure
plot(0:0.01:tfinal, p, '-r');
hold on
plot(0:0.01:tfinal, v, '-b');