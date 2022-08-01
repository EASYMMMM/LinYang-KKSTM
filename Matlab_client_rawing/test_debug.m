% close all,clear all;clc;
% warning('off');
% ip='172.31.1.147'; % The IP of the controller
% % start a connection with the server
% global t_Kuka;
% t_Kuka=net_establishConnection( ip );
% total_Fx=[];total_Fy=[];total_Fz=[];
% 
% addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  
% 
% 
% if ~exist('t_Kuka','var') || isempty(t_Kuka) || strcmp(t_Kuka.Status,'closed')
%   warning('Connection could not be establised, script aborted');
%   return;
% else
%     f1=getEEFPos( t_Kuka )
%     f2=getEEF_Force(t_Kuka);
% end
% 
% net_turnOffServer( t_Kuka );
% fclose(t_Kuka);


close all;clear;clc;
warning('off')
%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR7R800; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object
addpath('C:\Lin YANG\from me\Motion-Planning-for-KUKA-LBR-main-oriiii2\Motion-Planning-for-KUKA-LBR-main')  
flag=iiwa.net_establishConnection();
if flag==0
  return;
end

f1=iiwa.getEEFPos()
f2=iiwa.getJointsExternalTorques()

iiwa.realTime_startDirectServoJoints();
pause(3);

f1=iiwa.getEEFPos()
f2=iiwa.getJointsExternalTorques()


iiwa.realTime_stopDirectServoJoints();
iiwa.net_turnOffServer()


