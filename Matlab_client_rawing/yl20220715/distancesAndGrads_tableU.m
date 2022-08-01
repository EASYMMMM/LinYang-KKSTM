function [dists,grads] = distancesAndGrads_tableU(q, this_obstacle, delta_table, myspace)
% obs should be 3*6, where 3*1 means [~, boundary] =
% find_distance(position, obs, myspace) and 
    %Distances of the control points from the obstacles along with their derivative computed in q
    expand=0.1;
    old_points = zeros(5,3);
    old_points(1,:) = [-(cos(q(1))*sin(q(2)))/5; -(sin(q(1))*sin(q(2)))/5; cos(q(2))/5 + 9/25];
    old_points(2,:) = [-(21*cos(q(1))*sin(q(2)))/50; -(21*sin(q(1))*sin(q(2)))/50; (21*cos(q(2)))/50 + 9/25];
    old_points(3,:) = [-(21*cos(q(1))*sin(q(2)))/50 - (sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))))/5 - (cos(q(1))*cos(q(4))*sin(q(2)))/5; (sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))))/5 - (21*sin(q(1))*sin(q(2)))/50 - (cos(q(4))*sin(q(1))*sin(q(2)))/5;(21*cos(q(2)))/50 + (cos(q(2))*cos(q(4)))/5 + (cos(q(3))*sin(q(2))*sin(q(4)))/5 + 9/25];
    old_points(4,:) = [-(21*cos(q(1))*sin(q(2)))/50 - (2*sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))))/5 - (2*cos(q(1))*cos(q(4))*sin(q(2)))/5;(2*sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))))/5 - (21*sin(q(1))*sin(q(2)))/50 - (2*cos(q(4))*sin(q(1))*sin(q(2)))/5;(21*cos(q(2)))/50 + (2*cos(q(2))*cos(q(4)))/5 + (2*cos(q(3))*sin(q(2))*sin(q(4)))/5 + 9/25];
    old_points(5,:) = [(21*sin(q(6))*(cos(q(5))*(cos(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) - cos(q(1))*sin(q(2))*sin(q(4))) + sin(q(5))*(cos(q(3))*sin(q(1)) + cos(q(1))*cos(q(2))*sin(q(3)))))/100 - (21*cos(q(6))*(sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))) + cos(q(1))*cos(q(4))*sin(q(2))))/100 - (21*cos(q(1))*sin(q(2)))/50 - (2*sin(q(4))*(sin(q(1))*sin(q(3)) - cos(q(1))*cos(q(2))*cos(q(3))))/5 - (2*cos(q(1))*cos(q(4))*sin(q(2)))/5;
                   (21*cos(q(6))*(sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) - cos(q(4))*sin(q(1))*sin(q(2))))/100 - (21*sin(q(1))*sin(q(2)))/50 - (21*sin(q(6))*(cos(q(5))*(cos(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))) + sin(q(1))*sin(q(2))*sin(q(4))) + sin(q(5))*(cos(q(1))*cos(q(3)) - cos(q(2))*sin(q(1))*sin(q(3)))))/100 + (2*sin(q(4))*(cos(q(1))*sin(q(3)) + cos(q(2))*cos(q(3))*sin(q(1))))/5 - (2*cos(q(4))*sin(q(1))*sin(q(2)))/5;
                                                                                                                                                                                           (21*cos(q(2)))/50 + (2*cos(q(2))*cos(q(4)))/5 + (21*sin(q(6))*(cos(q(5))*(cos(q(2))*sin(q(4)) - cos(q(3))*cos(q(4))*sin(q(2))) + sin(q(2))*sin(q(3))*sin(q(5))))/100 + (21*cos(q(6))*(cos(q(2))*cos(q(4)) + cos(q(3))*sin(q(2))*sin(q(4))))/100 + (2*cos(q(3))*sin(q(2))*sin(q(4)))/5 + 9/25];
                                                                                                                                                                                       
                                                                                                                                                                                       
     
                                                                                                                                                                          
    boundarys=zeros(3,6);
     dists = zeros(6,1);
     points=old_points;
     points(:,1)=-old_points(:,1);
     points(:,2)=-old_points(:,2);
     
    for i=1:5
        [distance, boundary] = find_distance(points(i,:)', this_obstacle, myspace, 0);
        dists(i) = distance;
        boundarys(:,i) = boundary;
    end
    
    
%     find_distance(points(i,:), this_obstacle, myspace, expand)
    
    
    
    samples=6;
    delta_table2=2*delta_table/samples;
    min_l=1000;
    for each = 1:samples
        
        this_point_table=points(5,:)'+delta_table2*each;
        [distance_t, this_boundary_t] = find_distance(this_point_table, this_obstacle, myspace, 0);
        if min_l>distance_t
            min_l=distance_t;
            points_table=this_point_table;
            boundary_t=this_boundary_t;
        end
    end
%
%     single=[0.35; -0.6; 0.3]
%     center=[0.475; -0.6; 0.2;];
%     distan=norm(single-center)
    
%     points
%     find_distance(single, 35, myspace, 0);
    
    dists(6) = min_l;
    boundarys(:,6) = boundary_t;


    grads = zeros(6,7);
    p0=expand;
    [J67, A_mat_products] = Jacobian_table(q,1);
    J=J67(1:3,:);
    J_pinv = pinv(J);
    for oo = 1:6
        if dists(oo)>expand
            F3=zeros(3,1);
        else
            if oo == 6
%   points_table
%     boundarys(:,6)
% (1/dists(oo)-1/p0)*1/dists(oo)^2
                F3=(1/dists(oo)-1/p0)*1/dists(oo)^2*(points_table-boundarys(:,6))/norm(points_table-boundarys(:,6));
            else
                F3=(1/dists(oo)-1/p0)*1/dists(oo)^2*(points(oo,:)'-boundarys(:,oo))/norm(points(oo,:)'-boundarys(:,oo));
            end
        end
        torque=J_pinv*F3;
        grads(oo,:)=torque;
    end
    
end

