% Vasilis Belagiannis - CAMP - TUM - belagian@in.tum.de
function RT = getRT(rx, ry, rz, tx , ty, tz)

r1 = cos(ry)*cos(rz);

r2 = (cos(rz)*sin(rx)*sin(ry)) - (cos(rx)*sin(rz));

r3 = (sin(rx)*sin(rz)) + (cos(rx)*cos(rz)*sin(ry));

r4 = cos(ry)*sin(rz);

r5 = (sin(rx)*sin(ry)*sin(rz)) + (cos(rx)*cos(rz));

r6 = (cos(rx)*sin(ry)*sin(rz)) - (cos(rz)*sin(rx));

r7 = -sin(ry);

r8 = cos(ry)*sin(rx);

r9 = cos(rx)*cos(ry);

R = [ r1 r2 r3; r4 r5 r6; r7 r8 r9];

RT = zeros(3,4);

RT(1:3,1:3) = R;
RT(1,4) = tx;
RT(2,4) = ty;
RT(3,4) = tz;
