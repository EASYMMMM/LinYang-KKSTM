function out = auto_save(ddd)
current_now=datestr(now); 
out =[];
for i =1:length(current_now)
    current_now(i)
    if current_now(i) ~= ':' && current_now(i) ~= ' '
        out=[out current_now(i)];
    end
end

end