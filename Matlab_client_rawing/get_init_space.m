function data = get_init_space(where_robot_3,angle7)
theta=atand(where_robot_3(2)/where_robot_3(1)); 
% all_theta=theta-angle7*180/pi;
all_theta=angle7;
data = cell(12,4);L=0.1;
% height_z_lower=0.11;
height_z_lower=0.285;
height_z_higher=0.4;
x_big=0.785;
x_small=0.685;
if abs(angle7)<0.1

data{1,3}=[x_big,0.1,height_z_lower];
data{4,3}=[x_small,0.1,height_z_lower];      

data{2,3}=[x_big,0,height_z_lower];
data{5,3}=[x_small,0,height_z_lower];

data{3,3}=[x_big,-0.1,height_z_lower];
data{6,3}=[x_small,-0.1,height_z_lower];


data{7,3}=[x_big,0.1,height_z_higher];
data{10,3}=[x_small,0.1,height_z_higher];      

data{8,3}=[x_big,0,height_z_higher];
data{11,3}=[x_small,0,height_z_higher];

data{9,3}=[x_big,-0.1,height_z_higher];
data{12,3}=[x_small,-0.1,height_z_higher];

elseif abs(where_robot_3(1)-x_small)>abs(where_robot_3(1)-x_big)
    % in state  7 8 9
data{1,3}=[where_robot_3(1),abs(where_robot_3(2)),height_z_lower];
data{4,3}=[where_robot_3(1)-cosd(all_theta)*L,abs(where_robot_3(2))-sind(all_theta)*L,height_z_lower];

data{2,3}=[x_big,0,height_z_lower];
data{5,3}=[x_small,0,height_z_lower];

data{3,3}=[where_robot_3(1),-abs(where_robot_3(2)),height_z_lower];
data{6,3}=[where_robot_3(1)-cosd(all_theta)*L,-abs(where_robot_3(2))-sind(all_theta)*L,height_z_lower];


data{7,3}=[where_robot_3(1),abs(where_robot_3(2)),height_z_higher];
data{10,3}=[where_robot_3(1)-cosd(all_theta)*L,abs(where_robot_3(2))-sind(all_theta)*L,height_z_higher];    

data{8,3}=[x_big,0,height_z_higher];
data{11,3}=[x_small,0,height_z_higher];

data{9,3}=[where_robot_3(1),-abs(where_robot_3(2)),height_z_higher];
data{12,3}=[where_robot_3(1)-cosd(all_theta)*L,-abs(where_robot_3(2))-sind(all_theta)*L,height_z_higher];

elseif abs(where_robot_3(1)-x_small)<abs(where_robot_3(1)-x_big)
data{4,3}=[where_robot_3(1),abs(where_robot_3(2)),height_z_lower];
data{1,3}=[where_robot_3(1)+cosd(all_theta)*L,abs(where_robot_3(2))+sind(all_theta)*L,height_z_lower];

data{2,3}=[x_big,0,height_z_lower];
data{5,3}=[x_small,0,height_z_lower];

data{6,3}=[where_robot_3(1),-abs(where_robot_3(2)),height_z_lower];
data{3,3}=[where_robot_3(1)+cosd(all_theta)*L,-abs(where_robot_3(2))+sind(all_theta)*L,height_z_lower];


data{10,3}=[where_robot_3(1),abs(where_robot_3(2)),height_z_higher];
data{7,3}=[where_robot_3(1)+cosd(all_theta)*L,abs(where_robot_3(2))+sind(all_theta)*L,height_z_higher];    

data{8,3}=[x_big,0,height_z_higher];
data{11,3}=[x_small,0,height_z_higher];

data{12,3}=[where_robot_3(1),-abs(where_robot_3(2)),height_z_higher];
data{9,3}=[where_robot_3(1)+cosd(all_theta)*L,-abs(where_robot_3(2))+sind(all_theta)*L,height_z_higher];
 
end

%define data as the dictionary, where 1st is keys, 2nd is parameters of space 
% 3th is via point. 4th is feasibility. 
for ii = 1:12 
    data{ii,1} = ii;
    data{ii,2} = ii;
    data{ii,4} = 0;
end

end
