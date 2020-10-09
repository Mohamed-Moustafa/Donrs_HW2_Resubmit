%%symbolic
syms q1 q2 q3 q4 q5 q6 a1 a2 a3 d1 d2 d3 d3_dash delta_q

%% Forward Kinematics for Our Robot
H_trans_symbolic=Rz(q1)*Tx(a1)*Tz(d1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
H_rot_symbolic=Rx(q4)*Ry(q5)*Rx(q6)*Tx(a3);
H_symbolic=H_trans_symbolic*H_rot_symbolic; % EndEffector Homogenous Matrix
H_symbolic=simplify(H_symbolic)
%non symbolic Forward Kinematics
a1=25;a2=25;d1=400;d2=560;d3=515;d3_dash=sqrt(d3^2+a2^2);a3=0;
delta_q=atan(a2/d3);


syms q1 q2 q3 q4 q5 q6
%symbolic but a1 and fixed paramters subsitued
H_trans_symbolic=Rz(q1)*Tx(a1)*Tz(d1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
H_rot_symbolic=Rx(q4)*Ry(q5)*Rx(q6)*Tx(a3);
H_symbolic=H_trans_symbolic*H_rot_symbolic; % EndEffector Homogenous Matrix
H_symbolic=simplify(H_symbolic)
q=[pi/3 pi/3 pi/3 0 0 0] % give our angles to robot
q1=q(1);q2=q(2);q3=q(3);q4=q(4);q5=q(5);q6=q(6);

%subsitute with angles in Forward Kinematics Equations

%Translation Part
H_trans=Rz(q1)*Tz(d1)*Tx(a1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4)*Ry(q5)*Rx(q6)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H=H_trans*H_rot;



%%inverse kinematics
px=H(1,4);py=H(2,4);pz=H(3,4);

%Case 1&2:
q1=atan2(py,px); % q1

ax=a1*cos(q1);
ay=a1*sin(q1);
d=pz-d1;
s=sqrt((px-ax)^2+(py-ay)^2);
D=sqrt(s^2+d^2);

phi=atan2(d,s);
psi= acos((-(d3_dash^2)+d2^2+D^2)/(2*d2*D));
psi2= acos((-D^2+d3_dash^2+d2^2)/(2*d3_dash*d2));
q2=pi/2-(phi+psi); %q2
q3=pi/2-psi2+delta_q; %q3
H_trans=Rz(q1)*Tz(d1)*Tx(a1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);

H_456_1=inv(H_trans)*H;


%%%
nx=H_456_1(1,1);
ny=H_456_1(2,1);
nz=H_456_1(3,1);

sx=H_456_1(1,2);
sy=H_456_1(2,2);
sz=H_456_1(3,2);

ax=H_456_1(1,3);
ay=H_456_1(2,3);
az=H_456_1(3,3);


if nx ~= 1
    q4_1=atan2(ny,-nz);
    q4_2=atan2(-ny,nz);
    q6_1=atan2(sx,ax);
    q6_2=atan2(-sx,-ax);
    if ax~=0
        q5_1=atan2(ax/(cos(q6)),nx);
        q5_2=atan2(-ax/(cos(q6)),nx);
    else
        q5_1=atan2(sx/(sin(q6)),nx);
        q5_2=atan2(-sx/(sin(q6)),nx);
    end
else
    q5=acos(nx);
    q5_1=q5;
    q5_2=q5;
end
%%%
fprintf('1st solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1,q2,q3,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1)*Tz(d1)*Tx(a1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H1=H_trans*H_rot
fprintf('2nd solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1,q2,q3,q4_2,q5_2,q6_2);

%Translation Part
H_trans=Rz(q1)*Tz(d1)*Tx(a1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_2)*Ry(q5_2)*Rx(q6_2)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H2=H_trans*H_rot

%Case 3&4 :
q1_2=q1;
q2_2=q2+2*psi;
q3_2=-(3*pi/2)+psi2+delta_q;
H_trans=Rz(q1_2)*Tz(d1)*Tx(a1)*Ry(q2_2)*Tz(d2)*Ry(q3_2-delta_q)*Tx(d3_dash)*Ry(delta_q);

H_456_2=H_trans/H;


%%%
nx=H_456_2(1,1);
ny=H_456_2(2,1);
nz=H_456_2(3,1);

sx=H_456_2(1,2);
sy=H_456_2(2,2);
sz=H_456_2(3,2);

ax=H_456_2(1,3);
ay=H_456_2(2,3);
az=H_456_2(3,3);


if nx ~= 1
    q4_1=atan2(ny,-nz);
    q4_2=atan2(-ny,nz);
    q6_1=atan2(sx,ax);
    q6_2=atan2(-sx,-ax);
    if ax~=0
        q5_1=atan2(ax/(cos(q6)),nx);
        q5_2=atan2(-ax/(cos(q6)),nx);
    else
        q5_1=atan2(sx/(sin(q6)),nx);
        q5_2=atan2(-sx/(sin(q6)),nx);
    end
else
    q5=acos(nx);
    q5_1=q5;
    q5_2=q5;
end
%%%
%%%
fprintf('3rd solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_2,q2_2,q3_2,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1_2)*Tz(d1)*Tx(a1)*Ry(q2_2)*Tz(d2)*Ry(q3_2-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H3=H_trans*H_rot
fprintf('4th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_2,q2_2,q3_2,q4_2,q5_2,q6_2);

%Translation Part
H_trans=Rz(q1_2)*Tz(d1)*Tx(a1)*Ry(q2_2)*Tz(d2)*Ry(q3_2-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_2)*Ry(q5_2)*Rx(q6_2)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H4=H_trans*H_rot

%case 5&6

q1=q1+pi;
ax=a1*cos(q1);
ay=a1*sin(q1);
d=pz-d1; s=sqrt((px-ax)^2+(py-ay)^2); D=sqrt(s^2+d^2);

phi=atan2(d,s);
psi= acos((-(d3_dash^2)+d2^2+D^2)/(2*d2*D));
psi2= acos((-D^2+d3_dash^2+d2^2)/(2*d3_dash*d2));
%q1=atan2(py,(px+a1))

q2=pi/2-(phi+psi);
q3=pi/2-psi2+delta_q;

q1_3=q1;
q2_3=-q2;
q3_3=-(3*pi/2)+psi2+delta_q;
H_trans=Rz(q1_3)*Tz(d1)*Tx(a1)*Ry(q2_3)*Tz(d2)*Ry(q3_3-delta_q)*Tx(d3_dash)*Ry(delta_q);


H_456_3=H_trans/H;


%%%
nx=H_456_3(1,1);
ny=H_456_3(2,1);
nz=H_456_3(3,1);

sx=H_456_3(1,2);
sy=H_456_3(2,2);
sz=H_456_3(3,2);

ax=H_456_3(1,3);
ay=H_456_3(2,3);
az=H_456_3(3,3);


if nx ~= 1
    q4_1=atan2(ny,-nz);
    q4_2=atan2(-ny,nz);
    q6_1=atan2(sx,ax);
    q6_2=atan2(-sx,-ax);
    if ax~=0
        q5_1=atan2(ax/(cos(q6)),nx);
        q5_2=atan2(-ax/(cos(q6)),nx);
    else
        q5_1=atan2(sx/(sin(q6)),nx);
        q5_2=atan2(-sx/(sin(q6)),nx);
    end
else
    q5=acos(nx);
    q5_1=q5;
    q5_2=q5;
end
%%%

%%%
fprintf('5th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_3,q2_3,q3_3,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1_3)*Tz(d1)*Tx(a1)*Ry(q2_3)*Tz(d2)*Ry(q3_3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H5=H_trans*H_rot
fprintf('6th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_3,q2_3,q3_3,q4_2,q5_2,q6_2);

%Translation Part
H_trans=Rz(q1_3)*Tz(d1)*Tx(a1)*Ry(q2_3)*Tz(d2)*Ry(q3_3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_2)*Ry(q5_2)*Rx(q6_2)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H6=H_trans*H_rot


%Case 7&8:
q1_4=q1;
q2_4=q2_3-2*psi;
q3_4=+pi/2-psi2+delta_q;
H_trans=Rz(q1_4)*Tz(d1)*Tx(a1)*Ry(q2_4)*Tz(d2)*Ry(q3_4-delta_q)*Tx(d3_dash)*Ry(delta_q);

H_456_4=H_trans/H;

%%%
nx=H_456_4(1,1);
ny=H_456_4(2,1);
nz=H_456_4(3,1);

sx=H_456_4(1,2);
sy=H_456_4(2,2);
sz=H_456_4(3,2);

ax=H_456_4(1,3);
ay=H_456_4(2,3);
az=H_456_4(3,3);


if nx ~= 1
    q4_1=atan2(ny,-nz);
    q4_2=atan2(-ny,nz);
    q6_1=atan2(sx,ax);
    q6_2=atan2(-sx,-ax);
    if ax~=0
        q5_1=atan2(ax/(cos(q6)),nx);
        q5_2=atan2(-ax/(cos(q6)),nx);
    else
        q5_1=atan2(sx/(sin(q6)),nx);
        q5_2=atan2(-sx/(sin(q6)),nx);
    end
else
    q5=acos(nx);
    q5_1=q5;
    q5_2=q5;
end
%%%
%%%

%%%
fprintf('7th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_4,q2_4,q3_4,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1_4)*Tz(d1)*Tx(a1)*Ry(q2_4)*Tz(d2)*Ry(q3_4-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H5=H_trans*H_rot
fprintf('8th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_4,q2_4,q3_4,q4_2,q5_2,q6_2);

%Translation Part
H_trans=Rz(q1_4)*Tz(d1)*Tx(a1)*Ry(q2_4)*Tz(d2)*Ry(q3_4-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_2)*Ry(q5_2)*Rx(q6_2)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H6=H_trans*H_rot
