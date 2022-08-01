function [q,R,series_state via_points which_state space]= RL2m3m3(where_robot_3,last_space,CHANGE,lastq,lastR,num_obs,slope_IMU)

% which_state=3;



if CHANGE == 0
    space=get_init_space_table(0);
    if num_obs == 1
        all_space=cell2mat(space(:,3));
        [y,which_state]=min(sum(abs(all_space-where_robot_3.'),2));
        
        R=ones(48,48)*-inf;
        GESTURE=1;
        R_goal=100;
        GOAL=11;
        
        OBSTALE=[];
        feas=cell2mat(space(:,4));
        which=find(feas==1);
        for w =1:length(which)
            w=which(w);
            num=space{w,1};
            OBSTALE=[OBSTALE num];
        end
        
        for i=1:12
            R(i,i+12)=-3; %double up
            R(i,i+24)=-2; %H up R stay
            R(i,i+36)=-1; %R up H stay
            if i-4>0
                R(i,i-4)=-1;%forward
            end
            if i+4<13
                R(i,i+4)=-1;%backward
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;%left
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;%right
            end
        end
        
        for i=13:24
            R(i,i-12)=-1;  %double down
            if i-4>12
                R(i,i-4)=-1;
            end
            if i+4<25
                R(i,i+4)=-1;
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;
            end
        end
        
        for i=25:36
            R(i,i-24)=-1; %H down robot stay
            if i-4>24
                R(i,i-4)=-1;
            end
            if i+4<37
                R(i,i+4)=-1;
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;
            end
        end
        
        for i=37:48
            R(i,i-36)=-1; %R down H stay
            if i-4>36
                R(i,i-4)=-1;
            end
            if i+4<49
                R(i,i+4)=-1;
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;
            end
        end
        
        
        
        R(10,11)=R_goal;R(23,11)=R_goal;R(12,11)=R_goal;R(7,11)=R_goal;
        R(35,11)=R_goal;R(47,11)=R_goal;
        
        for obs0 = 1:length(OBSTALE)  % layer 1 2
            obs=OBSTALE(obs0);
            for i=1:48
                R(i,obs)=-inf;
            end
        end
        
        for obs0 = 1:length(OBSTALE)% layer 3
            obs=OBSTALE(obs0);
            if obs<=12
                obs=obs+24;
                for i=1:48
                    R(i,obs)=-inf;
                end
            end
        end
        
        R(41,42)=-1;
        R(43,42)=-1;
        R(38,42)=-1;
        R(46,42)=-1;
        
        for obs0 = 1:length(OBSTALE)% layer 4
            obs=OBSTALE(obs0);
            if obs>=13
                obs=obs+24;
                for i=1:48
                    R(i,obs)=-inf;
                end
            end
        end
        
        
        for obs0 = 1:48
            R(obs0,8)=-inf;
        end
        for obs0 = 1:48
            R(obs0,44)=-inf;
            R(obs0,43)=-inf;
        end
        % reinforcement learning parameters
        gamma=1;
        q=zeros(size(R));     % q matrix
        q1=ones(size(R))*inf; % previous q matrix
        count=0;
        
        % % visualize obstacle
        % axis([0,3,0,6]);
        % hold on;
        % plot([1,3],[4,4],'g','linewidth',2);
        % plot([1,3],[5,5],'g','linewidth',2);
        % plot([1,1],[4,5],'g','linewidth',2);
        
        % % intial state
        % y=randperm(3);
        % state=y(1);
        state=which_state;greedy=0.5;
        
        
        % q learning
        
        for episode=0:50000
            w=rand(1);
            qma=max(q(state,:));
            if qma~=0 && w>greedy
                x=find(q(state,:)==qma);
            else
                x=find(R(state,:)>=-10);
            end
            % choose action
            if size(x,1)>0
                x1=RandomPermutation(x);
                x1=x1(1);
            end
            % update q matrix
            qMax=max(q,[],2);
            q(state,x1)=R(state,x1)+gamma*qMax(x1);
            % break if converged: small deviation on q for 1000 consecutive
            if sum(sum(abs(q1-q)))<0.0001 && sum(sum(q))>190
                if count>500,
                    episode        % report last episode
                    break          % for
                else
                    count=count+1; % set counter if deviation of q is small
                end
            else
                q1=q;
                count=0;
            end
            
            if(R(state,x1)==R_goal)
                y=randperm(3);
                state=3;
                %          pause(0.4);
            else
                state=x1;
            end
        end
        
        %normalization
        g=max(max(q));
        if g>0
            q=R_goal*q/g;
        end
    end
    
    
    
    
    
    %%%%% OBS=2
    if num_obs == 2
        all_space=cell2mat(space(:,3));
        [y,which_state]=min(sum(abs(all_space-where_robot_3.'),2));
        
        R=ones(48,48)*-inf;
        GESTURE=1;
        R_goal=100;
        GOAL=11;
        
        OBSTALE=[];
        feas=cell2mat(space(:,4));
        which=find(feas==1);
        for w =1:length(which)
            w=which(w);
            num=space{w,1};
            OBSTALE=[OBSTALE num];
        end
        
        for i=1:12
            R(i,i+12)=-3; %double up
            R(i,i+24)=-2; %H up R stay
            R(i,i+36)=-1; %R up H stay
            if i-4>0
                R(i,i-4)=-1;%forward
            end
            if i+4<13
                R(i,i+4)=-1;%backward
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;%left
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;%right
            end
        end
        
        for i=13:24
            R(i,i-12)=-1;  %double down
            if i-4>12
                R(i,i-4)=-1;
            end
            if i+4<25
                R(i,i+4)=-1;
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;
            end
        end
        
        for i=25:36
            R(i,i-24)=-1; %H down robot stay
            if i-4>24
                R(i,i-4)=-1;
            end
            if i+4<37
                R(i,i+4)=-1;
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;
            end
        end
        
        for i=37:48
            R(i,i-36)=-1; %R down H stay
            if i-4>36
                R(i,i-4)=-1;
            end
            if i+4<49
                R(i,i+4)=-1;
            end
            if mod(i,4)~=1
                R(i,i-1)=-1;
            end
            if mod(i,4)~=0;
                R(i,i+1)=-1;
            end
        end
        
        
        
        R(10,11)=R_goal;R(23,11)=R_goal;R(12,11)=R_goal;R(7,11)=R_goal;
        R(35,11)=R_goal;R(47,11)=R_goal;
        
        for obs0 = 1:length(OBSTALE)  % layer 1 2
            obs=OBSTALE(obs0);
            for i=1:48
                R(i,obs)=-inf;
            end
        end
        
        for obs0 = 1:length(OBSTALE)% layer 3
            obs=OBSTALE(obs0);
            if obs<=12
                obs=obs+24;
                for i=1:48
                    R(i,obs)=-inf;
                end
            end
        end
        
        R(41,42)=-1;
        R(43,42)=-1;
        R(38,42)=-1;
        R(46,42)=-1;
        
        for obs0 = 1:length(OBSTALE)% layer 4
            obs=OBSTALE(obs0);
            if obs>=13
                obs=obs+24;
                for i=1:48
                    R(i,obs)=-inf;
                end
            end
        end
        
        
        for obs0 = 1:48
            R(obs0,8)=-inf;
        end
        for obs0 = 1:48
            R(obs0,44)=-inf;
            R(obs0,43)=-inf;
        end
        % reinforcement learning parameters
        gamma=1;
        q=zeros(size(R));     % q matrix
        q1=ones(size(R))*inf; % previous q matrix
        count=0;
        
        % % visualize obstacle
        % axis([0,3,0,6]);
        % hold on;
        % plot([1,3],[4,4],'g','linewidth',2);
        % plot([1,3],[5,5],'g','linewidth',2);
        % plot([1,1],[4,5],'g','linewidth',2);
        
        % % intial state
        % y=randperm(3);
        % state=y(1);
        state=which_state;greedy=0.5;
        
        
        % q learning
        
        for episode=0:50000
            w=rand(1);
            qma=max(q(state,:));
            if qma~=0 && w>greedy
                x=find(q(state,:)==qma);
            else
                x=find(R(state,:)>=-10);
            end
            % choose action
            if size(x,1)>0
                x1=RandomPermutation(x);
                x1=x1(1);
            end
            % update q matrix
            qMax=max(q,[],2);
            q(state,x1)=R(state,x1)+gamma*qMax(x1);
            % break if converged: small deviation on q for 1000 consecutive
            if sum(sum(abs(q1-q)))<0.0001 && sum(sum(q))>190
                if count>500,
                    episode        % report last episode
                    break          % for
                else
                    count=count+1; % set counter if deviation of q is small
                end
            else
                q1=q;
                count=0;
            end
            
            if(R(state,x1)==R_goal)
                y=randperm(3);
                state=3;
                %          pause(0.4);
            else
                state=x1;
            end
        end
        
        %normalization
        g=max(max(q));
        if g>0
            q=R_goal*q/g;
        end
    end
    
    
    %% 如果不需要变更环境
else
    space=last_space;
    all_space=cell2mat(space(:,3));
    [y,which_state]=min(sum(abs(all_space-where_robot_3.'),2));
    q=lastq;
    R=lastR;
end
s=which_state;
%s=28;
series_state=[s];
i=1;


while s~=11
    
    %     %商
    %     Y(i)=5.5-floor((s-1)/3);
    %     %余数
    %     X(i)=0.5+rem(s-1,3);
    %     %plot(X,Y,'*');
    %
    %     A=plot([X(i)-0.5,X(i)+0.5],[Y(i)-0.5,Y(i)-0.5],'r-','linewidth',2);
    %     B=plot([X(i)-0.5,X(i)+0.5],[Y(i)+0.5,Y(i)+0.5],'r-','linewidth',2);
    %     C=plot([X(i)-0.5,X(i)-0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
    %     D=plot([X(i)+0.5,X(i)+0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
    %     pause(0.2);
    %     if i>1;
    %     plot([X(i-1),X(i)],[Y(i-1),Y(i)],'b-','linewidth',2);
    %     end
    
    qm=max(q(s,:));
    if qm~=0
        ac=find(q(s,:)==qm);
    else
        ac=find(R(s,:)>=0);
    end
    
    if size(ac,2)>1
        act=RandomPermutation(ac);
        act=act(1);
    else
        act=ac;
    end
    
    %         delete(A);
    %         delete(B);
    %         delete(C);
    %         delete(D);
    series_state=[series_state act];
    s=act;
    i=i+1;
end

% %商
% Y(i)=5.5-floor((s-1)/3);
% %余数
% X(i)=0.5+rem(s-1,3);
% A=plot([X(i)-0.5,X(i)+0.5],[Y(i)-0.5,Y(i)-0.5],'r-','linewidth',2);
% B=plot([X(i)-0.5,X(i)+0.5],[Y(i)+0.5,Y(i)+0.5],'r-','linewidth',2);
% C=plot([X(i)-0.5,X(i)-0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
% D=plot([X(i)+0.5,X(i)+0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
%
% if i>1;
%     plot([X(i-1),X(i)],[Y(i-1),Y(i)],'b-','linewidth',2);
% end



%%
disp('my solution is')
series_state
via_points=[];
for num_next =1:length(series_state)
    next=series_state(num_next);
    
    next=mod(next,24);
    if next==0
        next=24;
    end
    fir=cell2mat(space(:,1));
    temp_via=find(fir==next);
    pos=space{temp_via,3}.';
    via_points=[via_points pos];
end


end