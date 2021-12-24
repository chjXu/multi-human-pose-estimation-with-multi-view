% Vasilis Belagiannis - CAMP - TUM - belagian@in.tum.de

clear all; close all; clc;

N=5;

data=cell(1,N);
for cam=1:N
    data{cam}=dlmread(sprintf('Camera%d.cal',cam-1));
end


for cam=1:5
    P{cam}=data{cam}(1:3,1:4);
    K{cam}=data{cam}(5:7,1:3);
    R{cam}=data{cam}(8:10,1:3);
    T{cam}=data{cam}(11,1:3)';
    %RT{cam}=[R{cam} T{cam}];
    %Pcomp{cam}=K{cam}*RT{cam};
    dlmwrite(sprintf('P%d.txt',cam-1),P{cam});
end

save('prjectionMat','P');