
function [force_x,force_y,force_z]=key_control(KEY_CONTROL)

    if KEY_CONTROL==1
        w = waitforbuttonpress;
        if w
            p = get(gcf, 'CurrentCharacter');
        switch p
           case 30,
               disp('上');
                force_x=0;
                force_y=0;
                force_z=5*9.8;
           case 31,
               disp('下');
                force_x=0;
                force_y=0;
                force_z=-5*9.8;
           case 28,
               disp('左');
                force_x=-5*9.8;
                force_y=-5*9.8;
                force_z=0*9.8;
           case 29,
               disp('右');
                force_x=5*9.8;
                force_y=5*9.8;
                force_z=0*9.8;
            otherwise
                disp('else');
                force_x=0;
                force_y=0;
                force_z=0;
            end
        end
    else
    force_x=0*9.8;
    force_y=0*9.8;
    force_z=0*9.8;
    end

end
     