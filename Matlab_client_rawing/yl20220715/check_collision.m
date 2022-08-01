function isTrajectoryInCollision = check_collision(point_collision)
% Create two platforms
platform1 = collisionBox(0.4,0.4,0.4);  %It should be trasformed from camera
platform1.Pose = trvec2tform([0.7 0 0.2]);  % position of center of obstacles
% Store in a cell array for collision-checking
worldCollisionArray = {platform1};
robot = loadrobot("kukaIiwa14","DataFormat","column","Gravity",[0 0 -9.81]);
rng(0);
ik = inverseKinematics("RigidBodyTree",robot);
weights = ones(1,6);
%%
% point_2=[-0.4013;0.8987;0;-0.3310;0;1.9106;-0.3927];
% point_above=[   -0.1748;0.6793;0;-1.0330;0;1.4280;-0.3927];
% point_collision=[   -0.1748;0.7447;0;-1.1795;0;1.2161;-0.3927];

q=point_collision;
% Initialize outputs
inCollision = false(length(q), 1); % Check whether each pose is in collision
worldCollisionPairIdx = cell(length(q),1); % Provide the bodies that are in collision

i=1;
tic
    [inCollision(i),sepDist] = checkCollision(robot,q(:,i),worldCollisionArray,"IgnoreSelfCollision","on","Exhaustive","on");
    
    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx]; 
    worldCollisionPairIdx{i} = worldCollidingPairs;
    toc
% end
isTrajectoryInCollision = any(inCollision);


end