function [v_direction]= which_refer_v(total_act)
now=total_act(1);
next=total_act(2);
if now-next == 1
    v_direction = 1;
end
if now-next == 3
    v_direction = 1;
end
if now-next == 6
    v_direction = -1;
end

if now-next == -1
    v_direction = -1;
end
if now-next == -3
    v_direction = -1;
end
if now-next == -6
    v_direction = 1;
end

end