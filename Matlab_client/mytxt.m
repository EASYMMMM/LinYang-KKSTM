function state = mytxt(A)

    filename = 'C:\Lin YANG\from me\KUKA\KUKA_Matlab\KST-Kuka-Sunrise-Toolbox-master\YL_1.txt';
    file_id = fopen(filename,'a+');
    [hang lie]=size(A);
    for jj = 1:hang
        fprintf(file_id,'%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\t%.10f\r\n',A(jj,:));
    end
    fclose(file_id);
    state = 1;
end