
gonna=all_y_filtered(intent1);
v=[0];v_temp=[];
for each_y = 1:length(gonna)-1
     this_y = gonna(each_y);
     next_y = gonna(each_y+1);
    if abs(next_y - this_y) > 10
        v_add=0;

    else
        v_add=next_y - this_y;
    end
v=[v v_add];
end
