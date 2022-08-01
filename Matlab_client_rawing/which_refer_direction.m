function [direction]= which_refer_direction(total_act)
now=total_act(1);
next=total_act(2);
if abs(now-next) == 1
    direction = 2;
end
if abs(now-next) == 3
    direction = 1;
end
if abs(now-next) == 6
    direction = 3;
end

end