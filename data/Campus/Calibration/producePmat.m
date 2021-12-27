% Vasilis Belagiannis - CAMP - TUM - belagian@in.tum.de

clear all; close all; clc;

%original size
width=720;
height=576;

scale=0.5;

%Camera 0 int - ext
focal=2.014732e+01;
kappa1=4.518309e-04;
cx=3.707192e+02;
cy=2.785074e+02;
sx=2.300000e-02;
K1 = getK(focal,sx,cx,cy,scale);

tx=-1.787557e+00;
ty=1.361094e+00;
tz=5.226973e+00;
rx=1.684894e+00;
ry=-6.059471e-03;
rz=-1.412313e-02;
RT1 = getRT(rx, ry, rz, tx , ty, tz);

P{1} = K1*RT1;

%Camera 1 int - ext
focal=1.978153e+01;
kappa1=4.185620e-04;
cx=3.681166e+02;
cy=2.614934e+02;
sx=2.300000e-02;
K2 = getK(focal,sx,cx,cy,scale);

tx=4.922850e+00;
ty=1.161417e+00;
tz=6.684942e+00;
rx=-2.788873e+00;
ry=-1.431726e+00;
rz=-1.911579e+00;
RT2 = getRT(rx, ry, rz, tx , ty, tz);

P{2} = K2*RT2;


%Camera 2 int - ext
focal=3.224534e+01;
kappa1=-8.921162e-05;
cx=3.351895e+02;
cy=2.841090e+02;
sx=2.300000e-02;
K3 = getK(focal,sx,cx,cy,scale);

tx=-4.901276e+00;
ty=5.298783e-01;
tz=1.120239e+01;
rx=1.813738e+00;
ry=9.891695e-01;
rz=1.975452e-01;
RT3 = getRT(rx, ry, rz, tx , ty, tz);

P{3} = K3*RT3;

save('prjectionMat','P');
dlmwrite('P0.txt', P{1},'delimiter', ' ');
dlmwrite('P1.txt', P{2},'delimiter', ' ');
dlmwrite('P2.txt', P{3},'delimiter', ' ');

K = cell(1,3);
K{1} = K1; K{2} = K2; K{3} = K3;
m_RT = cell(1,3);
m_RT{1} = RT1; m_RT{2} = RT2; m_RT{3} = RT3;
save('intrinsic.mat','K');
save('m_RT.mat', 'm_RT');
save('P.mat', 'P');
save('prjectionMat','P');
