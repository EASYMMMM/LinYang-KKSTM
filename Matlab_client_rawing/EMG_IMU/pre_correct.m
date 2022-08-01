function pre_corr = pre_correct(each_com_use, intent) % a very simple updating api, assumes defaults, will need to be modified for nondefaults
        if intent == 5 || intent == 6
            intent_pos = 5;
            intent_other = 6;
        end
        if intent == 1 || intent == 2
            intent_pos = 1;
            intent_other = 2;
        end
        if intent == 3 || intent == 4
            intent_pos = 3;
            intent_other = 4;
        end
        
 pre_corr=[intent];
            for j = 1:size(each_com_use,1)-1
                this=each_com_use(j,1);
                next=each_com_use(j+1,1);
                if next-this>0
                    pre_corr=[pre_corr; intent_pos;];
                else
                    pre_corr=[pre_corr; intent_other;];
                end
            end
end