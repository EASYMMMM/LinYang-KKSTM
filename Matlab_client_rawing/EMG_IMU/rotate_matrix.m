function R = rotate_matrix(this)
% fixed angle
    a=this(15);
    b=this(14);
    y=this(13);
%     raw_Ax=this1(13);
%     raw_Ay=this1(14);
%     raw_Az=this1(15);
%     Ax = kalmanx(raw_Ax);
%     Ay = kalmany(raw_Ay);
%     Az = kalmanz(raw_Az);
%     filter_A=[filter_A [Ax; Ay; Az;]];
    
%     A=[raw_Ax; raw_Ay; raw_Az;];
    Rz=[[cosd(a) -sind(a) 0]; [sind(a) cosd(a) 0]; [0 0 1];];
    Ry=[[cosd(b) 0 sind(b)]; [0 1 0]; [-sind(b) 0 cosd(b)];];
    Rx=[[1 0 0]; [0 cosd(y) -sind(y)]; [0 sind(y) cosd(y)];];
    R=Rz*Ry*Rx;

end