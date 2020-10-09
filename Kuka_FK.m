%%symbolic
syms q1 q2 q3 q4 q5 q6 a1 a2 a3 d1 d2 d3 d3_dash delta_q

%% Forward Kinematics for Our Robot
H_trans_symbolic=Rz(q1)*Tx(a1)*Tz(d1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
H_rot_symbolic=Rx(q4)*Ry(q5)*Rx(q6)*Tx(a3);
H_symbolic=H_trans_symbolic*H_rot_symbolic; % EndEffector Homogenous Matrix
H_symbolic=simplify(H_symbolic)