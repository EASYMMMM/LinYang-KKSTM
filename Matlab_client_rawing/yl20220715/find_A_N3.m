function [final_A]= find_A_N3(loop_i)

init_A=[];
for i = 1:loop_i+3
    row = zeros(1,loop_i+3);
    if i == 1
        row(i) = 1;
    else
        row(i) = 1;row(i-1) = -1;
    end
 init_A=[init_A;row;];  
end
final_A=init_A*init_A*init_A;
[hang lie] = size(final_A);
final_A=final_A(:,1:(lie-3));

end