
  
  way_points=[[0.6400; 0.0403; 0.1;] [0.7250; 0.093; 0.1-0.01;]];

  [points,points_dot,points_dotdot,pp] = cubicpolytraj(way_points,tpts,tvec,...
                'VelocityBoundaryCondition', zeros(3,2));  
            
            
%%     

    q = zeros(n, length(t));
    qd = zeros(n, length(t));
    qdd = zeros(n, length(t));
    
        derivativeBreaks = robotics.core.internal.changeEndSegBreaks(modBreaks, t);

        % Get coefficients for first derivative pp-form and evaluate
        dCoeffs = robotics.core.internal.polyCoeffsDerivative(modCoeffs);
        ppd = mkpp(derivativeBreaks, dCoeffs, n);
        qd(:,:) = ppval(ppd, t);
        
        
        
%%


[hang,liehh]=size(points);    
theta_points=[0,0.772629915332932,0,-1.26703950381033,0,1.10192323444653,0].';

for i_inv = 1 : liehh
    init_theta1=180;
    init_theta2=0;
    xd=points(1,i_inv);
    yd=points(2,i_inv);
    zd=points(3,i_inv);
    [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
    count_no=0;
            while isempty(All_theta)
                    if yd<0
                        init_theta2=init_theta2+1;
                        init_theta1=180-count_no;
                    elseif yd>0
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;
                    else
                        init_theta2=init_theta2+1;
                        init_theta1=180+count_no;  
                    end
                count_no=count_no+1;
                [All_theta] = inverse_with_gesture(xd,yd,zd,init_theta1,init_theta2).';
            end
    [hang,lie]=size(All_theta);
    temp=theta_points(:,end);
    tott=1000;
    delta_matrix=All_theta-temp;
    for each_lie =1:lie
        now=sum(abs(All_theta(:,each_lie)-temp));
        if now < tott
            tott=now;
            which=each_lie;
        end
    end
    to_add=All_theta(:,which);
    if round <= 20
        to_add(7)=0;
    elseif round >= 20 && lock == 1
        to_add(7)=angle_comb*180/pi;
    end
    theta_points=[theta_points to_add];
end
theta_points=theta_points.*pi/180;
theta_points(:,1)=[];
refer_pos=[];

for q=1:size(theta_points,2)
         [ poseo, nsparam, rconf, jout ] = ForwardKinematics( theta_points(:,q).', robot_type );
        end_effector_po = poseo(1:3,4);
        refer_pos=[refer_pos end_effector_po];
end



figure; plot(points(1,:),points(2,:),'-b')
figure; plot(refer_pos(1,:),refer_pos(2,:),'-b')
