

B= [ 0.0009446918438402,  0.00188938368768,0.0009446918438402 ];
A= [1,   -1.911197067426,   0.9149758348014];




after_raw_EMG3=all_9xyz(1:100000,2);
onebyone=[];y=all_9xyz(1:3,2);

for oo = 1: 100000-2
    x=(after_raw_EMG3(oo:oo+3-1,1));
    y_new=0;
    for i = 1:length(B)
          y_new = y_new + (B(i) * x(end-i+1));
    end
    for i = 2:length(B)
          y_new = y_new - (A(i) * y(end-i+2));
    end
    y=[y; y_new;];
    
end

figure(90)
plot(after_raw_EMG3(:,1)); hold on; plot(y(:,1)); 
legend('ori','direct filter','real time filter')