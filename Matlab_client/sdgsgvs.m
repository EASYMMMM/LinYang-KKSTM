% close all;clear;clc;
data = [2:5];
for m= 1:length(data)
name_string = ['s' num2str(m) '=data(m)'];
eval(name_string)
end

 