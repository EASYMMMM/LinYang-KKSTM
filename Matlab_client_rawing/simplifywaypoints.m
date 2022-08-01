function [new_way_points]= simplifywaypoints(way_points,total_act)
new_way_points=[way_points];
for z = length(total_act)-2
    this = total_act(z);
    next= total_act(z+1);
    nextt=total_act(z+2);
    delta1 = next - this;
    delta2 = nextt - next;
    if delta1 == delta2
        new_way_points=[new_way_points(:,1:z) new_way_points(:,z+1:end)];
    end
end