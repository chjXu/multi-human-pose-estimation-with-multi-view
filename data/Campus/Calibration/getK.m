% Vasilis Belagiannis - CAMP - TUM - belagian@in.tum.de
function K = getK(f,s,cx,cy,scale)

K = zeros(3,3);

K(1,1)=(f/s)*scale;
K(2,2)=(f/s)*scale;
K(1,3)=cx*scale;
K(2,3)=cy*scale;
K(3,3)=1;