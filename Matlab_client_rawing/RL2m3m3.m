function [total_act via_points which_state space]= RL2m3m3(where_robot_3,angle7,up,lock,last_space,laststate,round)

% close all;clear;clc;
% warning('off')
% where_robot_3=[0.55,0.05,0.1].';angle7=-0.2542;

if lock == 0
    space=get_init_space(where_robot_3,angle7);
else
    space=last_space;
end

via_points=[];total_act=[];
all_space=cell2mat(space(:,3));
if round > 1
    last_all_space=cell2mat(last_space(:,3));
    last_all_space([2 5 8 11],:)=100*ones(4,3);
end




if laststate == 1 || laststate == 4 || laststate == 3 || laststate == 6
    [y,which_state]=min(sum(abs(last_all_space-where_robot_3.'),2));
    if which_state <= 6
        if laststate == 1 || laststate == 4
            if which_state <=3
                which_state = 1;
            elseif which_state > 3
                which_state = 4;
            end
        elseif laststate == 3 || laststate == 6
            if which_state <=3
                which_state = 3;
            elseif which_state > 3
                which_state = 6;
            end
        end
    end
else
    [y,which_state]=min(sum(abs(all_space-where_robot_3.'),2));
end


% up level, up =0 means going down

if up == 1 && which_state <= 6
    total_act = [which_state which_state+6];
else
    if which_state>=7
        goal=which_state - 6;
    else
        if which_state<=3
            goal = which_state+3;
        else
            goal = which_state - 3;
        end
    end
    total_act = [which_state goal];
end
total_act;
for o = 1:2
    fir=cell2mat(space(:,1));
    temp_via=find(fir==total_act(o));
    pos=space{temp_via,3}.';
    via_points=[via_points pos];
end
    
end