
function q = Kuka_IK_fn(H)
%% in this function i took care of singularites , space limits , and angles it will just display one any found
% it will display singualrity when singularity found
% also for space limits there are 2 possiple space limits 
% there's limitation that not any configuration can reach (absolute space limit)
% there's another limitation that only four solution are possible (at the border of the workspace)
% Also i take into consideration joints limits

px=H(1,4);
py=H(2,4);
pz=H(3,4);


a1=25;a2=25;d1=400;d2=560;d3=515;d3_dash=sqrt(d3^2+a2^2);a3=0;
delta_q=atan(a2/d3);

distance= d2+d3_dash+a3 ;
absolute_limit= sqrt((px-25)^2+py^2 + (pz-400)^2 );
almost_limit = sqrt((px+25)^2+ py^2 + (pz-400)^2 );

if absolute_limit <= distance

    
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

H_456_1=H_trans\H;


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
    q5_1=atan2(sqrt(ax^2+sx^2),nx);
    q5_2=atan2(-sqrt(ax^2+sx^2),nx);
    if ((q5_1 > -1e-5) && (q5_1 < 1e-5) ) || ((q5_2 > -1e-5) && (q5_2 < 1e-5))
    disp('singularity');
    end
else
disp('singularity');
end
%%%
angles1=[q1 ,q2 ,q3 ,q4_1 , q5_1 ,q6_1];
fprintf('1st solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1,q2,q3,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1)*Tz(d1)*Tx(a1)*Ry(q2)*Tz(d2)*Ry(q3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H1=H_trans*H_rot
angles2=[q1 ,q2 ,q3 ,q4_2 , q5_2 ,q6_2];
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
H_456_2=H_trans\H;


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
    q5_1=atan2(sqrt(ax^2+sx^2),nx);
    q5_2=atan2(-sqrt(ax^2+sx^2),nx);
    if ((q5_1 > -1e-5) && (q5_1 < 1e-5) ) || ((q5_2 > -1e-5) && (q5_2 < 1e-5))
    disp('singularity');
    end
else
disp('singularity');
end
%%%
%%%
angles3=[q1_2 ,q2_2 ,q3_2 ,q4_1 , q5_1 ,q6_1];
fprintf('3rd solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_2,q2_2,q3_2,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1_2)*Tz(d1)*Tx(a1)*Ry(q2_2)*Tz(d2)*Ry(q3_2-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H3=H_trans*H_rot


angles4=[q1_2 ,q2_2 ,q3_2 ,q4_2 , q5_2 ,q6_2];

fprintf('4th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_2,q2_2,q3_2,q4_2,q5_2,q6_2);

%Translation Part
H_trans=Rz(q1_2)*Tz(d1)*Tx(a1)*Ry(q2_2)*Tz(d2)*Ry(q3_2-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_2)*Ry(q5_2)*Rx(q6_2)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H4=H_trans*H_rot




if almost_limit <= distance

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


H_456_3=H_trans\H;


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
    q5_1=atan2(sqrt(ax^2+sx^2),nx);
    q5_2=atan2(-sqrt(ax^2+sx^2),nx);
    if ((q5_1 > -1e-5) && (q5_1 < 1e-5) ) || ((q5_2 > -1e-5) && (q5_2 < 1e-5))
    disp('singularity');
    end
else
disp('singularity');
end
%%%
angles5=[q1_3 ,q2_3 ,q3_3 ,q4_1 , q5_1 ,q6_1];

%%%
fprintf('5th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_3,q2_3,q3_3,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1_3)*Tz(d1)*Tx(a1)*Ry(q2_3)*Tz(d2)*Ry(q3_3-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H5=H_trans*H_rot


angles6=[q1_2 ,q2_2 ,q3_2 ,q4_2 , q5_2 ,q6_2];

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

H_456_4=H_trans\H;

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
    q5_1=atan2(sqrt(ax^2+sx^2),nx);
    q5_2=atan2(-sqrt(ax^2+sx^2),nx);
    if ((q5_1 > -1e-5) && (q5_1 < 1e-5) ) || ((q5_2 > -1e-5) && (q5_2 < 1e-5))
    disp('singularity');
    end
else
disp('singularity');
end
%%%
%%%
angles7=[q1_4 ,q2_4 ,q3_4 ,q4_1 , q5_1 ,q6_1];

%%%
fprintf('7th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_4,q2_4,q3_4,q4_1,q5_1,q6_1);
%Translation Part
H_trans=Rz(q1_4)*Tz(d1)*Tx(a1)*Ry(q2_4)*Tz(d2)*Ry(q3_4-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_1)*Ry(q5_1)*Rx(q6_1)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H7=H_trans*H_rot


angles8=[q1_4 ,q2_4 ,q3_4 ,q4_2 , q5_2 ,q6_2];

fprintf('8th solution IK angles   q1=%f  , q2=%f  , q3=%f, q4=%f  , q5=%f  , q6=%f \n \n Matrix for these angles',q1_4,q2_4,q3_4,q4_2,q5_2,q6_2);

%Translation Part
H_trans=Rz(q1_4)*Tz(d1)*Tx(a1)*Ry(q2_4)*Tz(d2)*Ry(q3_4-delta_q)*Tx(d3_dash)*Ry(delta_q);
%Rotation Part
H_rot=Rx(q4_2)*Ry(q5_2)*Rx(q6_2)*Tx(a3);
%Full EndEffector Homogoneous Matrix
H8=H_trans*H_rot

q1_limit= (angles1(1) < -170*pi/180) || (angles1(1) > 170*pi/180);
q2_limit= (angles1(2) < -190*pi/180) || (angles1(2) > 45*pi/180);
q3_limit= (angles1(3) < -120*pi/180) || (angles1(3) > 156*pi/180);
q4_limit= (angles1(4) < -185*pi/180) || (angles1(4) > 185*pi/180);
q5_limit= (angles1(5) < -120*pi/180) || (angles1(5) > 120*pi/180);
q6_limit= (angles1(6) < -350*pi/180) || (angles1(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of first solution angles exceed limits');
end

q1_limit= (angles2(1) < -170*pi/180) || (angles2(1) > 170*pi/180);
q2_limit= (angles2(2) < -190*pi/180) || (angles2(2) > 45*pi/180);
q3_limit= (angles2(3) < -120*pi/180) || (angles2(3) > 156*pi/180);
q4_limit= (angles2(4) < -185*pi/180) || (angles2(4) > 185*pi/180);
q5_limit= (angles2(5) < -120*pi/180) || (angles2(5) > 120*pi/180);
q6_limit= (angles2(6) < -350*pi/180) || (angles2(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of second solution angles exceed limits');
end

q1_limit= (angles3(1) < -170*pi/180) || (angles3(1) > 170*pi/180);
q2_limit= (angles3(2) < -190*pi/180) || (angles3(2) > 45*pi/180);
q3_limit= (angles3(3) < -120*pi/180) || (angles3(3) > 156*pi/180);
q4_limit= (angles3(4) < -185*pi/180) || (angles3(4) > 185*pi/180);
q5_limit= (angles3(5) < -120*pi/180) || (angles3(5) > 120*pi/180);
q6_limit= (angles3(6) < -350*pi/180) || (angles3(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of third solution angles exceed limits');
end

q1_limit= (angles4(1) < -170*pi/180) || (angles4(1) > 170*pi/180);
q2_limit= (angles4(2) < -190*pi/180) || (angles4(2) > 45*pi/180);
q3_limit= (angles4(3) < -120*pi/180) || (angles4(3) > 156*pi/180);
q4_limit= (angles4(4) < -185*pi/180) || (angles4(4) > 185*pi/180);
q5_limit= (angles4(5) < -120*pi/180) || (angles4(5) > 120*pi/180);
q6_limit= (angles4(6) < -350*pi/180) || (angles4(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of fourth solution angles exceed limits');
end

q1_limit= (angles5(1) < -170*pi/180) || (angles5(1) > 170*pi/180);
q2_limit= (angles5(2) < -190*pi/180) || (angles5(2) > 45*pi/180);
q3_limit= (angles5(3) < -120*pi/180) || (angles5(3) > 156*pi/180);
q4_limit= (angles5(4) < -185*pi/180) || (angles5(4) > 185*pi/180);
q5_limit= (angles5(5) < -120*pi/180) || (angles5(5) > 120*pi/180);
q6_limit= (angles5(6) < -350*pi/180) || (angles5(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of fifth solution angles exceed limits');
end

q1_limit= (angles6(1) < -170*pi/180) || (angles6(1) > 170*pi/180);
q2_limit= (angles6(2) < -190*pi/180) || (angles6(2) > 45*pi/180);
q3_limit= (angles6(3) < -120*pi/180) || (angles6(3) > 156*pi/180);
q4_limit= (angles6(4) < -185*pi/180) || (angles6(4) > 185*pi/180);
q5_limit= (angles6(5) < -120*pi/180) || (angles6(5) > 120*pi/180);
q6_limit= (angles6(6) < -350*pi/180) || (angles6(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of sixth solution angles exceed limits');
end

q1_limit= (angles7(1) < -170*pi/180) || (angles7(1) > 170*pi/180);
q2_limit= (angles7(2) < -190*pi/180) || (angles7(2) > 45*pi/180);
q3_limit= (angles7(3) < -120*pi/180) || (angles7(3) > 156*pi/180);
q4_limit= (angles7(4) < -185*pi/180) || (angles7(4) > 185*pi/180);
q5_limit= (angles7(5) < -120*pi/180) || (angles7(5) > 120*pi/180);
q6_limit= (angles7(6) < -350*pi/180) || (angles7(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of seventh solution angles exceed limits');
end

q1_limit= (angles8(1) < -170*pi/180) || (angles8(1) > 170*pi/180);
q2_limit= (angles8(2) < -190*pi/180) || (angles8(2) > 45*pi/180);
q3_limit= (angles8(3) < -120*pi/180) || (angles8(3) > 156*pi/180);
q4_limit= (angles8(4) < -185*pi/180) || (angles8(4) > 185*pi/180);
q5_limit= (angles8(5) < -120*pi/180) || (angles8(5) > 120*pi/180);
q6_limit= (angles8(6) < -350*pi/180) || (angles8(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of eigthath solution angles exceed limits');
end
else
    disp('Only Four Solution Can be Exist (At the borders of the workspace)');  
    angles5=[nan,nan,nan,nan,nan,nan];
    angles6=[nan,nan,nan,nan,nan,nan];
    angles7=[nan,nan,nan,nan,nan,nan];
    angles8=[nan,nan,nan,nan,nan,nan];
    q1_limit= (angles1(1) < -170*pi/180) || (angles1(1) > 170*pi/180);
q2_limit= (angles1(2) < -190*pi/180) || (angles1(2) > 45*pi/180);
q3_limit= (angles1(3) < -120*pi/180) || (angles1(3) > 156*pi/180);
q4_limit= (angles1(4) < -185*pi/180) || (angles1(4) > 185*pi/180);
q5_limit= (angles1(5) < -120*pi/180) || (angles1(5) > 120*pi/180);
q6_limit= (angles1(6) < -350*pi/180) || (angles1(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of first solution angles exceed limits');
end

q1_limit= (angles2(1) < -170*pi/180) || (angles2(1) > 170*pi/180);
q2_limit= (angles2(2) < -190*pi/180) || (angles2(2) > 45*pi/180);
q3_limit= (angles2(3) < -120*pi/180) || (angles2(3) > 156*pi/180);
q4_limit= (angles2(4) < -185*pi/180) || (angles2(4) > 185*pi/180);
q5_limit= (angles2(5) < -120*pi/180) || (angles2(5) > 120*pi/180);
q6_limit= (angles2(6) < -350*pi/180) || (angles2(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of second solution angles exceed limits');
end

q1_limit= (angles3(1) < -170*pi/180) || (angles3(1) > 170*pi/180);
q2_limit= (angles3(2) < -190*pi/180) || (angles3(2) > 45*pi/180);
q3_limit= (angles3(3) < -120*pi/180) || (angles3(3) > 156*pi/180);
q4_limit= (angles3(4) < -185*pi/180) || (angles3(4) > 185*pi/180);
q5_limit= (angles3(5) < -120*pi/180) || (angles3(5) > 120*pi/180);
q6_limit= (angles3(6) < -350*pi/180) || (angles3(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of third solution angles exceed limits');
end

q1_limit= (angles4(1) < -170*pi/180) || (angles4(1) > 170*pi/180);
q2_limit= (angles4(2) < -190*pi/180) || (angles4(2) > 45*pi/180);
q3_limit= (angles4(3) < -120*pi/180) || (angles4(3) > 156*pi/180);
q4_limit= (angles4(4) < -185*pi/180) || (angles4(4) > 185*pi/180);
q5_limit= (angles4(5) < -120*pi/180) || (angles4(5) > 120*pi/180);
q6_limit= (angles4(6) < -350*pi/180) || (angles4(6) > 350*pi/180);
if q1_limit || q2_limit ||q3_limit ||q4_limit ||q5_limit ||q6_limit
    disp('one of fourth solution angles exceed limits');
end

end

else 
        disp('Workspace limit reached (Absolute Space Limit)');
        angles1=[nan,nan,nan,nan,nan,nan];
        angles2=[nan,nan,nan,nan,nan,nan];
        angles3=[nan,nan,nan,nan,nan,nan];
        angles4=[nan,nan,nan,nan,nan,nan];
        angles5=[nan,nan,nan,nan,nan,nan];
        angles6=[nan,nan,nan,nan,nan,nan];
        angles7=[nan,nan,nan,nan,nan,nan];
        angles8=[nan,nan,nan,nan,nan,nan];
end


q=[angles1;angles2;angles3;angles4;angles5;angles6;angles7;angles8];
end
