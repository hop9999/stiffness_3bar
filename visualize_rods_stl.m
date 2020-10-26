function visualize_rods_stl(rods, colors, alpha, scale)

fv1 = main_stlread('cylindb.stl');

if nargin < 4
scale = [0.3 0 0
    0 0.3 0
    0 0 29];
end

for index = 1:length(rods)
    
    rod_array = rods{index};
    for i = 1:(length(rod_array) / 6)
        shift = [(rod_array((i - 1)*6 + 1)+rod_array((i - 1)*6 + 4))/2 ,...
            (rod_array((i - 1)*6 + 2)+rod_array((i - 1)*6 + 5))/2,...
            (rod_array((i - 1)*6 + 3)+rod_array((i - 1)*6 + 6))/2];
        
        dx = (rod_array((i - 1)*6 + 4)-rod_array((i - 1)*6 + 1));
        dy = (rod_array((i - 1)*6 + 5)-rod_array((i - 1)*6 + 2));
        dz = (rod_array((i - 1)*6 + 6)-rod_array((i - 1)*6 + 3));
        
        angle1 = atan2(sqrt(dx^2 + dy^2),dz);
        angle2 = atan2(dy,dx);
        
        fv2.vertices = fv1.vertices*scale'*roty(180/pi*angle1)'*rotz(180/pi*angle2)' + shift;
        fv2.faces = fv1.faces;
        
        hold on
        patch(fv2, 'FaceColor', colors{index}, ...
            'EdgeColor', 'none','FaceAlpha',alpha);
    end
end
% 
% light('Position',[0.2 0 0.5],'Style','local');
% light('Position',[0.2 -1 1.5],'Style','local');
% light('Position',[0.2 1 1.5],'Style','local');
%     light('Position',[-2 3 4]*2,'Style','local');
%     light('Position',[1 -2 0.5]*2,'Style','local');

end