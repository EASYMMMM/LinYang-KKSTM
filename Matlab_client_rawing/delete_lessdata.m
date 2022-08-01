function [out1 out2] = delete_lessdata(all_round_test_X,all_round_test_Y)
all_intent=[];
for trival = 1:size(all_round_test_X,1)-1
    this_intent = all_round_test_X(trival,end);
    next_intent = all_round_test_X(trival+1,end);
    if this_intent ~= next_intent
        all_intent=[all_intent this_intent];
    end
end
all_intent=[all_intent next_intent];
num_all_intent=1:length(all_intent);
count_intent=1; after_all_test_X=[];

for trival2 = 1:size(all_round_test_X,1)
    this_intent = all_round_test_X(trival2,end);
    if this_intent ~= all_intent(count_intent)
        count_intent=count_intent+1;
    end
    after_all_test_X=[after_all_test_X; [all_round_test_X(trival2,1:end-1) count_intent]; ];
end


for nmb=num_all_intent
    nmb;
    which_hang=find(after_all_test_X(:,end) == nmb);
    howmany=length(which_hang);
    if howmany<100
        after_all_test_X(which_hang,:)=[];
        all_round_test_X(which_hang,:)=[];
        all_round_test_Y(which_hang,:)=[];
    end
end
out1=all_round_test_X;
out2=all_round_test_Y;
end