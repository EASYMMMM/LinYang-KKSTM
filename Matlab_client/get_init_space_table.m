function data = get_init_space_table(input)

% l是木板长度，abh是障碍的长宽高,x是base到障碍的距离
q=input;
% clc;
% clear all;

%%
platform1 = collisionBox(0.4,0.4,0.4);  %It should be trasformed from camera
platform1.Pose = trvec2tform([0.7 0 0.2]);  % position of center of obstacles
% Store in a cell array for collision-checking
worldCollisionArray = {platform1};
robot = loadrobot("kukaIiwa14","DataFormat","column","Gravity",[0 0 -9.81]);
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);


l=0.4;a=0.4;b=0.4;h=0.4;x=0.7;

plus_a=0.01;plus_b=0.15;plus_h=0.1;

data = cell(24,4);die_dueto_col=0;
%define data as the dictionary, where 1st is keys, 2nd is parameters of space 
% 3th is via point. 4th is feasibility. 
for ii = 1:24 
    data{ii,1} = ii;
end
%%
delta_a159=0.05;delta_h1=h+plus_h;delta_h2=0.05;delta_b5=b+2*plus_b;delta_b1=0.05;
delta_b9=0.05;delta_a2610=a/2;delta_a3711=a/2;delta_a4812=0.05;
% if x <= plus_b
%     delta_b7=0;
% else
%     delta_b7=x-plus_b;
% end
% 
% if l > a
%     delta_a369=0;
%     delta_a258=a+plus_a;
% else
%      delta_a369=a-l;
%      delta_a258=l+plus_a;
% end

%%
data{1,2}=[delta_a159,delta_b1,delta_h1];
data{2,2}=[delta_a2610,delta_b1,delta_h1];
data{3,2}=[delta_a3711,delta_b1,delta_h1];
data{4,2}=[delta_a4812,delta_b1,delta_h1];


data{5,2}=[delta_a159,delta_b5,delta_h1];
data{6,2}=[delta_a2610,delta_b5,delta_h1];
data{7,2}=[delta_a3711,delta_b5,delta_h1];
data{8,2}=[delta_a4812,delta_b5,delta_h1];

data{9,2}=[delta_a159,delta_b9,delta_h1];
data{10,2}=[delta_a2610,delta_b9,delta_h1];
data{11,2}=[delta_a3711,delta_b9,delta_h1];
data{12,2}=[delta_a4812,delta_b9,delta_h1];

for j = 13:24
    j_each=data{j-12,2};
    j_a=j_each(1);j_b=j_each(2);
    data{j,2}=[j_a,j_b,delta_h2];
end

toc
%%  via points
for p = 1:24
    if p <= 12
%         p_h=data{p,2}(3)/2; % 第一层的高度
        p_h=0.32;
    else
        p_h=data{p-12,2}(3)+data{p,2}(3)/2;  % 第一层的高度
    end
    
    p_a=0;
    endd=mod(p,4);
    if endd==0
        loopp=0;
    else
        loopp=4-endd;
    end
    for each_a = 1: loopp
        p_a=p_a+data{p+each_a,2}(1);
    end
    p_a=data{p,2}(1)/2+p_a+x-b/2-delta_a4812;
%     if mod(p,4)==0
%         p_a=-data{p,2}(1)/2+x-b/2;
%     elseif mod(p,4)==1
%         p_a=data{p,2}(1)/2+x+b/2;
%     elseif mod(p,4)==2
%         p_a=p_a-data{p,2}(1)/2+x;        
%     elseif mod(p,4)==3
%         p_a=p_a-data{p,2}(1)/2+x;        
%     end    
        
    if (mod(p,12)>=4 && mod(p,12)<=8)
        p_b=0;
    end
    if (mod(p,12)>=1 && mod(p,12)<=4)
        p_b=-(b+plus_b+data{p,2}(2))/2;
    end   
    if (mod(p,12)>=9 && mod(p,12)<=12)
        p_b=(b+plus_b+data{p,2}(2))/2;
    end
      data{p,3}=[p_a,p_b,p_h];
      
%       plot3(p_a,p_b,p_h,'b.','MarkerSize',2);hold on;
end
%%  feasiblity


point_collision=[   -0.1748;0.7447;0;-1.1795;0;1.2161;-0.3927];
die_duetoout=0;catch_die=0;
for k = 1:24
    now_x=data{k,3}(1);
    now_y=data{k,3}(2);
    now_z=data{k,3}(3);

    init_theta1=180; init_theta2=0;

        [All_theta] = inverse_with_gesture(now_x,now_y,now_z,init_theta1,init_theta2);
        

All_theta=All_theta.'*pi/180;
    if isempty(All_theta)
        data{k,4}=1;
    else 
        if All_theta(1,1) == -180
            All_theta(:,1)=[];
        end
        data{k,4}=0;
    end
    if k == 6 || k == 7
        data{k,4}=1;
    end

%%


% toc
if ismember(k,[3 4 7 8 11 12])
        q=All_theta(:,1);
        % Initialize outputs
        inCollision = false(length(q), 1); % Check whether each pose is in collision
        worldCollisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision

        i=1;

        [inCollision(i),sepDist] = checkCollision(robot,q(:,i),worldCollisionArray,"IgnoreSelfCollision","on","Exhaustive","on");
        [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
        worldCollidingPairs = [bodyIdx,worldCollisionObjIdx]; 
        worldCollisionPairIdx{i} = worldCollidingPairs;

%         isTrajectoryInCollision = any(inCollision);
        
        if any(inCollision) 
            data{k,4}=1;
        end
end


end
    



end
