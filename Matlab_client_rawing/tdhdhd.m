% 1 减少 2 增加
record_x1=record;
record_x2=flipud(record);

delta=((record_x1(1))-(record_x1(end)))/length(record_x1);
extend_max=(record_x1(1))+40*delta:-delta:(record_x1(1));
extend_min=(record_x1(end)):-delta:(record_x1(end))-40*delta;
after_X111=[extend_max.'; record_x1; extend_min(1:end-1).';]
after_Y111=[extend_max(2:end).'; record_x1; record_x1(end)*ones(41,1);]
% figure(89); plot(after_X111,after_Y111,'ro')

delta=((record_x2(1))-(record_x2(end)))/length(record_x2);
extend_max=(record_x2(1))+40*delta:-delta:(record_x2(1));
extend_min=(record_x2(end)):-delta:(record_x2(end))-40*delta;
after_X222=[extend_max.'; record_x2; extend_min(1:end-1).';]
after_Y222=[extend_max(2:end).'; record_x2;record_x2(end)*ones(41,1);]
% figure(88); plot(after_X222,after_Y222,'ro')


train_L_X111 = [after_X111 1*ones(size(after_X111,1),1)];
train_L_Y111 = after_Y111;
train_L_X222 = [after_X222 2*ones(size(after_X222,1),1)];
train_L_Y222 = after_Y222;


gprMdl_x25_int1 = fitrgp(train_L_X111,train_L_Y111);
gprMdl_x25_int2 = fitrgp(train_L_X222,train_L_Y222);

result111 = myGRP_onlyx_limitfuture(0.65, 1, gprMdl_x25_int1,200);
result_1=resubPredict(gprMdl_x25_int1);

figure; plot(result111)

result222 = myGRP_onlyx_limitfuture(0.55, 2, gprMdl_x25_int2,200);
result_2=resubPredict(gprMdl_x25_int2);
figure; plot(result222)

% x=train_L_X111(:,1);
% y=train_L_Y111;
% figure;plot(x,y,'o'); hold on;

train_X_xyz1=train_L_X222;
figure(42)
[ypred_f, ~, yci_f] = predict(gprMdl_x25_int2, train_X_xyz1 );

title('no data, not much confidence on the right');
hold on;
plot(train_X_xyz1(:,1) ,gprMdl_x25_int2.Y,'r.');
plot(train_X_xyz1(:,1) ,ypred_f, 'y.');
plot(train_X_xyz1(:,1), yci_f(:,1),'k:');
plot(train_X_xyz1(:,1), yci_f(:,2),'k:');
max_range=max(train_X_xyz1(:,1)); min_range=min(train_X_xyz1(:,1));
% jj=min_range-20:0.1:max_range+20;
% [ypred_f, ~, yci_f] = predict(gprMdl_x2, jj );
% plot(jj ,ypred_f, 'y.');

plot([min_range, max_range] , [min_range, max_range] ,'g')
