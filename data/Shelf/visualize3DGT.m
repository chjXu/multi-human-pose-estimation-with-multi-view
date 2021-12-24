% Vasilis Belagiannis - CAMP - TUM - belagian@in.tum.de

clear all; close all; clc;

images{1} = dir('Camera0/*.png');
images{2} = dir('Camera1/*.png');
images{3} = dir('Camera2/*.png');
images{4} = dir('Camera3/*.png');
images{5} = dir('Camera4/*.png');

load('actorsGT.mat');

%individual color
humanCol = {[1 0 0],[0 1 0],[1 0 1],[1 0.5 1]};

%calibration
load('Calibration/prjectionMat.mat');

N_cams=length(actor2D{1}{1});
N_frames=length(actor2D{1});


for fr=584:1:N_frames %frame
    
    %plot joints 3D
    subplot(3,3,N_cams+1);
    for person=1:1:length(actor2D) %person (1,2,3)
        xyz = actor3D{person}{fr}; %3D joints
        
        if size(xyz,1)>0
            c = repmat(humanCol{person},size(xyz,1),1);
            s = 50*ones(size(xyz,1),1);
            scatter3(xyz(:,1), xyz(:,2), xyz(:,3),s,c,'fill'); hold on;
          
            xlim([-5 5]); ylim([-5 5]); zlim([0 5]);
        end
    end
    hold off;
    
    
    %plot joints 2D
    for cam=1:1:N_cams
        
        %load image data
        im = imread([sprintf('Camera%d/',cam-1) images{cam}(fr).name]);
        subplot(3,3,cam);
        imshow(im); hold on;
        
        for person=1:1:length(actor2D) %person (1,2,3)
            
            xyz = actor3D{person}{fr}; %3D joints
            
            text(10,20,sprintf('Frame:%d, Camera:%d',fr,cam),'Color','r','FontSize',12);
            for m=1:1:size(xyz,1)
                
                X = [xyz(m,:) 1]';
                xy = P{cam}*X; %project the 3D point
                xy = xy./xy(3);
                
                text(xy(1),xy(2), num2str(m), 'Color', humanCol{person});
                
                if m==14
                    text(xy(1),xy(2)-30, sprintf('A%d',person), 'Color', humanCol{person}, 'FontSize', 12);
                end
            end
        end
        hold off;
    end
    pause(0.03);
end