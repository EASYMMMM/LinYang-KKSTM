close all,clear all;clc;
warning('off');
ip='172.31.1.147'; % The IP of the controller
% start a connection with the server
global t_Kuka;
t_Kuka=net_establishConnection( ip );
total_Fx=[];total_Fy=[];total_Fz=[];
f1=getEEFPos( t_Kuka );
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  


if ~exist('t_Kuka','var') || isempty(t_Kuka) || strcmp(t_Kuka.Status,'closed')
  warning('Connection could not be establised, script aborted');
  return;
else
%%


relVel=0.15; % the relative velocity
theta_points=[ -0.4013,0.9539,0,-1.0490,0,1.1374,-0.3927];
way_points=[[0.65;-0.3;0.25],[0.65;-0.3;0.525],[0.6;0;0.525],[0.65;0.3;0.525],[0.65;-0.3;0.25]];

theta_points2=num2cell(theta_points);

movePTPJointSpace( t_Kuka , theta_points2, relVel); % point to point motion in joint space


%%


for i = 1:100
    i
    f1=getJointsPos( t_Kuka );
    f2=getEEF_Force(t_Kuka);
    f_x=f2(1);
    f_y=f2(2);
    f_z=f2(3);
    total_Fx=[total_Fx f_x];
    total_Fy=[total_Fy f_y];
    total_Fz=[total_Fz f_z];
end


end

net_turnOffServer( t_Kuka );
fclose(t_Kuka);


figure(1)
plot(cell2mat(total_Fx))
figure(2)
plot(cell2mat(total_Fy))
figure(3)
plot(cell2mat(total_Fz))