function F = give_tableF(input,position_z)
if input == 1
    F=position_z+0.1;
elseif input == 0
    F=position_z;
elseif input == -1
    F=position_z-0;
end
end