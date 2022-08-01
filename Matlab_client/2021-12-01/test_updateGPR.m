

a=[1 2 3];
b=[8 9];
re=[1 2 4; 4 5 6;];
[X, Y] = meshgrid(a,b);
mesh(X,Y,re)
xlabel('percent for adding')
ylabel('rate of train to test')
zlabel( ' average MSE')

gprMdl_1
% all_MSE=[];denominator=3;result=0;
% for rate = 1:2
%     
%     all_MSE_thisrate=[];
%     for each_test = num_all_intent
%         each_test % 第几次
%         each_intent=all_intent(each_test);
%         which_each_test_file = find(after_all_test_X(:,end) == each_test);
%         each_test_file = after_all_test_X(which_each_test_file,:); % this is each trival, 14 with intent
%         % seperate data
%         bound=floor(size(each_test_file,1)*rate/denominator);
%         each_test_file_add=each_test_file(1:bound,:);
%         each_test_file_left=each_test_file(bound:end,:);
% 
%         each_test_file_Y = all_round_test_Y(which_each_test_file,1:end);
%         each_test_file_Y_add=each_test_file_Y(1:bound,:);
%         each_test_file_Y_left=each_test_file_Y(bound:end,:);       
%         % update GPR
%         tic
%         new_gprMdl_1 = updateGPRMdl(gprMdl_1,each_test_file_add, each_test_file_Y_add(:,1));
%         new_gprMdl_2 = updateGPRMdl(gprMdl_2,each_test_file_add, each_test_file_Y_add(:,2));
%         new_gprMdl_3 = updateGPRMdl(gprMdl_3,each_test_file_add, each_test_file_Y_add(:,3));
%         new_gprMdl_4 = updateGPRMdl(gprMdl_4,each_test_file_add, each_test_file_Y_add(:,4));
%         new_gprMdl_5 = updateGPRMdl(gprMdl_5,each_test_file_add, each_test_file_Y_add(:,5));
%         new_gprMdl_6 = updateGPRMdl(gprMdl_6,each_test_file_add, each_test_file_Y_add(:,6));
%         new_gprMdl_7 = updateGPRMdl(gprMdl_7,each_test_file_add, each_test_file_Y_add(:,7));
%         new_gprMdl_8 = updateGPRMdl(gprMdl_8,each_test_file_add, each_test_file_Y_add(:,8));
%         new_gprMdl_9 = updateGPRMdl(gprMdl_9,each_test_file_add, each_test_file_Y_add(:,9));
%         new_gprMdl_10 = updateGPRMdl(gprMdl_10,each_test_file_add, each_test_file_Y_add(:,10));
%         new_gprMdl_11 = updateGPRMdl(gprMdl_11,each_test_file_add, each_test_file_Y_add(:,11));
%         new_gprMdl_12 = updateGPRMdl(gprMdl_12,each_test_file_add, each_test_file_Y_add(:,12));
%         new_gprMdl_13 = updateGPRMdl(gprMdl_13,each_test_file_add, each_test_file_Y_add(:,13));
%         toc
%         
%         result = myGRP(each_test_file_left(:,1:end-1), each_intent, new_gprMdl_1,new_gprMdl_2,new_gprMdl_3,new_gprMdl_4,new_gprMdl_5,new_gprMdl_6,new_gprMdl_7,new_gprMdl_8,new_gprMdl_9,new_gprMdl_10,new_gprMdl_11,new_gprMdl_12,new_gprMdl_13);
%         mse = sum((each_test_file_Y_left(2:end,1:3) - result(:,1:3)).^2)./size(result,1);
%         avg_mse=sum(mse)/3;
%         all_MSE_thisrate=[all_MSE_thisrate avg_mse];
%     end
%     all_MSE=[all_MSE; all_MSE_thisrate;];
% end