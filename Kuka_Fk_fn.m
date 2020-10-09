function FK_matrix = Kuka_Fk_fn(angles)

q1=angles(1);q2=angles(2);q3=angles(3);q4=angles(4);q5=angles(5);q6=angles(6);

%non symbolic Forward Kinematics
a1=25;a2=25;d1=400;d2=560;d3=515;d3_dash=sqrt(d3^2+a2^2);a3=0;
delta_q=atan(a2/d3);
%% Forward Kinematics for Our Robot
H_trans=Rz(q1)*Tx(a1)*Tz(d1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
H_rot=Rx(q4)*Ry(q5)*Rx(q6)*Tx(a3);
FK_matrix=H_trans*H_rot; % EndEffector Homogenous Matrix


end