function [final_A]= find_A(loop_i)

init_A=[];
for i = 1:loop_i
    row = zeros(1,loop_i);
    if i == 1
        row(i) = 1;
    else
        row(i) = 1;row(i-1) = -1;
    end
 init_A=[init_A;row;];  
end
final_A=init_A*init_A*init_A;

A=[[1 0 0]; [-1 1 0]; [0 -1 1];];
A*A*A;

B=[[1 0 0 0 0]; [-1 1 0 0 0]; [0 -1 1 0 0]; [0 0 -1 1 0];[0 0 0 -1 1];];
B*B*B;
end