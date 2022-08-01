clc;
clear all;
% define state
R=ones(9,9)*-inf;  
for i=1:9
   if i-3>0
       R(i,i-3)=-1;
   end 
   if i+3<10
       R(i,i+3)=-1;
   end
   if mod(i,3)~=1
       R(i,i-1)=-1;
   end
   if mod(i,3)~=0;
       R(i,i+1)=-1;
   end
end
R(2,5)=-inf;R(4,5)=-inf;R(8,5)=-inf;R(6,5)=-inf;
R(3,6)=100;R(9,6)=100;

 
% reinforcement learning parameters
gamma=0.9;
q=zeros(size(R));     % q matrix
q1=ones(size(R))*inf; % previous q matrix
count=0;
 
% visualize obstacle 
axis([0,3,0,3]);
hold on;
% plot([1,3],[1,1],'g','linewidth',2);
% plot([1,3],[2,2],'g','linewidth',2);
% plot([1,1],[1,2],'g','linewidth',2);
 
% intial state
% y=randperm(3);
% state=y(1);
 state=4;
% q learning
tic
for episode=0:50000
 
    qma=max(q(state,:));
    if qma~=0
       x=find(q(state,:)==qma);
    else
       x=find(R(state,:)>=-1);
    end
    % choose action
    if size(x,1)>0
        x1=RandomPermutation(x);
        x1=x1(1);
    end
    % update q matrix
    qMax=max(q,[],2);
    q(state,x1)=R(state,x1)+gamma*qMax(x1);
    
%     Y(i)=2.5-floor((x1-1)/3);
%     X(i)=0.5+rem(x1-1,3);
%     % visualization
%     A=plot([X(i)-0.5,X(i)+0.5],[Y(i)-0.5,Y(i)-0.5],'r-','linewidth',2);
%     B=plot([X(i)-0.5,X(i)+0.5],[Y(i)+0.5,Y(i)+0.5],'r-','linewidth',2);
%     C=plot([X(i)-0.5,X(i)-0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
%     D=plot([X(i)+0.5,X(i)+0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
%     pause(0.05);

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
     
    if(R(state,x1)==100)
%          y=randperm(3);
%          state=y(1);
            state=4;
%          pause(0.4);
     else
         state=x1;
     end
%         delete(A);
%         delete(B);
%         delete(C);
%         delete(D);
 
end
toc
%normalization
g=max(max(q));
if g>0
    q=100*q/g;
end


%%

 
% visualize obstacle 
axis([0,3,0,3]);
hold on;
plot([1,2],[1,1],'g','linewidth',2);
plot([1,2],[2,2],'g','linewidth',2);
plot([1,1],[1,2],'g','linewidth',2);
plot([2,1],[2,2],'g','linewidth',2);
 plot([2,2],[1,2],'g','linewidth',2);
% st=randperm(3);
% s=st(1);
%s=28;
 s=4;
i=1;
while s~=6
 
    %商
    Y(i)=2.5-floor((s-1)/3)
    %余数
    X(i)=0.5+rem(s-1,3)
    %plot(X,Y,'*');
    
    A=plot([X(i)-0.5,X(i)+0.5],[Y(i)-0.5,Y(i)-0.5],'r-','linewidth',2);
    B=plot([X(i)-0.5,X(i)+0.5],[Y(i)+0.5,Y(i)+0.5],'r-','linewidth',2);
    C=plot([X(i)-0.5,X(i)-0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
    D=plot([X(i)+0.5,X(i)+0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
    pause(0.2);
    if i>1;
    plot([X(i-1),X(i)],[Y(i-1),Y(i)],'b-','linewidth',2);
    end
 
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
 
        delete(A);
        delete(B);
        delete(C);
        delete(D);
 
    s=act;
    i=i+1;
end
 
%商
Y(i)=2.5-floor((s-1)/3);
%余数
X(i)=0.5+rem(s-1,3);
A=plot([X(i)-0.5,X(i)+0.5],[Y(i)-0.5,Y(i)-0.5],'r-','linewidth',2);
B=plot([X(i)-0.5,X(i)+0.5],[Y(i)+0.5,Y(i)+0.5],'r-','linewidth',2);
C=plot([X(i)-0.5,X(i)-0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
D=plot([X(i)+0.5,X(i)+0.5],[Y(i)-0.5,Y(i)+0.5],'r-','linewidth',2);
 
if i>1;
    plot([X(i-1),X(i)],[Y(i-1),Y(i)],'b-','linewidth',2);
end
