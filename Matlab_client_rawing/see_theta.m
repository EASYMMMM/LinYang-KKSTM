

  theta=load('all_now_desired_theta.mat').all_now_desired_theta;
        for num_cube=1:size(all_cubes,1)/2
            Cube=all_cubes(2*num_cube-1:2*num_cube,:);
            % fill3([100 100 1000 1000],[-1000 1000 1000 -1000],[], 'r')
            fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)...
                Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3) Cube(1,3) Cube(1,3) Cube(1,3)], 'b'); hold on;
            fill3([Cube(1,1) Cube(1,1) Cube(1,1)+Cube(2,1) Cube(1,1)+Cube(2,1)], [Cube(1,2) Cube(1,2)+Cube(2,2)...
                Cube(1,2)+Cube(2,2) Cube(1,2) ], [Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3) Cube(1,3)+Cube(2,3)], 'b'); hold on;
        end
        
        
        
        
        for i= 1: length(theta)
            [X,~]=updateQ(theta(:,i)');
            plot3(X(1, 1), X(1, 2), X(1, 3), 'bo', 'markersize', 6);
            plot3(X(2, 1), X(2, 2), X(2, 3), 'ro', 'markersize', 6);
            plot3(X(3, 1), X(3, 2), X(3, 3), 'go', 'markersize', 6);
            plot3(X(4, 1), X(4, 2), X(4, 3), 'yo', 'markersize', 6);
            plot3(X(5, 1), X(5, 2), X(5, 3), 'ko', 'markersize', 6);
            plot3(X(6, 1), X(6, 2), X(6, 3), 'mo', 'markersize', 6);
            plot3(X(7, 1), X(7, 2), X(7, 3), 'bo', 'markersize', 6);
            plot3(X(8, 1), X(8, 2), X(8, 3), 'bo', 'markersize', 6);
            lynxServoSim(theta(1,i),theta(2,i),theta(3,i),theta(4,i),theta(5,i),theta(6,i),theta(7,i));
            % lynxServoSim(theta(:,i)');
            pause(0.01);
        end
        
        figure;
        plot(1:ite, Q_time, 'b', 'linewidth', 2);hold on;
        figure;
        plot(1:ite, RAR_time, 'r', 'linewidth', 2);