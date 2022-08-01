function [b,a,y] = inverse_angle(R)
% 弧度
b=atan2(-R(3,1) , (R(1,1)^2+R(2,1)^2)^0.5);
a = atan2( (R(2,1)/cos(b)) , (R(1,1)/cos(b)));
y = atan2( (R(3,2)/cos(b)) , (R(3,3)/cos(b)));

% 角度
a=a*180/pi;
b=b*180/pi;
y=y*180/pi;
end