function [positions, positions_right,center_left_image,center_right_image] = tracker2(width, zed, pos, target_sz, ...
	padding, kernel, lambda, output_sigma_factor, interp_factor, cell_size, ...
	features)
%TRACKER Kernelized/Dual Correlation Filter (KCF/DCF) tracking.
%   This function implements the pipeline for tracking with the KCF (by
%   choosing a non-linear kernel) and DCF (by choosing a linear kernel).
%
%   It is meant to be called by the interface function RUN_TRACKER, which
%   sets up the parameters and loads the video information.
%
%   Parameters:
%     WIDTH is the width of the stereocamera. 
%     ZED is the object of stereocamera.
%     POS and TARGET_SZ are the initial position and size of the target
%      (both in format [rows, columns]).
%     PADDING is the additional tracked region, for context, relative to 
%      the target size.
%     KERNEL is a struct describing the kernel. The field TYPE must be one
%      of 'gaussian', 'polynomial' or 'linear'. The optional fields SIGMA,
%      POLY_A and POLY_B are the parameters for the Gaussian and Polynomial
%      kernels.
%     OUTPUT_SIGMA_FACTOR is the spatial bandwidth of the regression
%      target, relative to the target size.
%     INTERP_FACTOR is the adaptation rate of the tracker.
%     CELL_SIZE is the number of pixels per cell (must be 1 if using raw
%      pixels).
%     FEATURES is a struct describing the used features (see GET_FEATURES).

%
%   Outputs:
%    POSITIONS is an Nx2 matrix of target positions over time (in the
%     format [rows, columns]).
%    TIME is the tracker execution time, without video loading/rendering.
%
%   Joao F. Henriques, 2014
all_9xyz=[];final_EMGxyz=[];
global b emg_show handle flag
flag = 0;count_which_matrix=0;

global READY
READY=0;

global each_update_16
each_update_16=[];

%% 发送字符串，pause（1）要不要都可以
% sendtxt = 'hello hello';
% fprintf(Client,sendtxt);
total_rece=[];count2=0;count_x=0;
total_rece_0=[];total_rece_1=[];total_rece_2=[];total_rece_6=[];ALL_LEN_OF_RECV2=[];
total_rece = [];total_head=0;total_rece_3=[];total_rece_4=[];total_rece_5=[];
global all_len_fun
all_len=[];all_len_fun=[];
%%
global DATA;
DATA=[];global all_len_fun_if
all_len_fun_if=[];
% CHANGE THIS TO THE IP OF THE COMPUTER RUNNING THE TRIGNO CONTROL UTILITY
HOST_IP = '10.11.0.223';
%%
%This example program communicates with the Delsys SDK to stream 16
%channels of EMG data and 48 channels of ACC data.



%% Create the required objects

%Define number of sensors
NUM_SENSORS = 16;

%handles to all plots
global plotHandlesEMG;
plotHandlesEMG = zeros(NUM_SENSORS,1);
global plotHandlesACC;
plotHandlesACC = zeros(NUM_SENSORS*3, 1);
global rateAdjustedEmgBytesToRead;

%TCPIP Connection to stream EMG Data
interfaceObjectEMG = tcpip(HOST_IP,50041);
interfaceObjectEMG.InputBufferSize = 6400;

%TCPIP Connection to stream ACC Data
interfaceObjectACC = tcpip(HOST_IP,50042);
interfaceObjectACC.InputBufferSize = 6400;

%TCPIP Connection to communicate with SDK, send/receive commands
commObject = tcpip(HOST_IP,50040);

%Timer object for drawing plots.
t = timer('Period', .1, 'ExecutionMode', 'fixedSpacing', 'TimerFcn', {@updatePlots, plotHandlesEMG});
global data_arrayEMG
data_arrayEMG = [];
global data_arrayACC
data_arrayACC = [];

%% Set up the plots
axesHandlesEMG = zeros(NUM_SENSORS,1);
axesHandlesACC = zeros(NUM_SENSORS,1);

%initiate the EMG figure
figureHandleEMG = figure('Name', 'EMG Data','Numbertitle', 'off',  'CloseRequestFcn', {@localCloseFigure, interfaceObjectEMG, interfaceObjectACC, commObject, t});
set(figureHandleEMG, 'position', [50 200 750 750])

for i = 1:NUM_SENSORS
    axesHandlesEMG(i) = subplot(4,4,i);

    plotHandlesEMG(i) = plot(axesHandlesEMG(i),0,'-y','LineWidth',1);

    set(axesHandlesEMG(i),'YGrid','on');
    %set(axesHandlesEMG(i),'YColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesEMG(i),'XGrid','on');
    %set(axesHandlesEMG(i),'XColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesEMG(i),'Color',[.15 .15 .15]);
    set(axesHandlesEMG(i),'YLim', [-.005 .005]);
    set(axesHandlesEMG(i),'YLimMode', 'manual');
    set(axesHandlesEMG(i),'XLim', [0 2000]);
    set(axesHandlesEMG(i),'XLimMode', 'manual');
    
    if(mod(i, 4) == 1)
        ylabel(axesHandlesEMG(i),'V');
    else
        set(axesHandlesEMG(i), 'YTickLabel', '')
    end
    
    if(i >12)
        xlabel(axesHandlesEMG(i),'Samples');
    else
        set(axesHandlesEMG(i), 'XTickLabel', '')
    end
    
    title(sprintf('EMG %i', i)) 
end

%initiate the ACC figure
figureHandleACC = figure('Name', 'ACC Data', 'Numbertitle', 'off', 'CloseRequestFcn', {@localCloseFigure, interfaceObjectEMG, interfaceObjectACC, commObject, t});
set(figureHandleACC, 'position', [850 200 750 750]);



for i= 1:NUM_SENSORS
    axesHandlesACC(i) = subplot(4, 4, i);
    hold on
    plotHandlesACC(i*3-2) = plot(axesHandlesACC(i), 0, '-y', 'LineWidth', 1);    
    plotHandlesACC(i*3-1) = plot(axesHandlesACC(i), 0, '-y', 'LineWidth', 1);   
    plotHandlesACC(i*3) = plot(axesHandlesACC(i), 0, '-y', 'LineWidth', 1);
    hold off 
    
    set(plotHandlesACC(i*3-2), 'Color', 'r')
    set(plotHandlesACC(i*3-1), 'Color', 'b')
    set(plotHandlesACC(i*3), 'Color', 'g')    
    set(axesHandlesACC(i),'YGrid','on');
    %set(axesHandlesACC(i),'YColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesACC(i),'XGrid','on');
    %set(axesHandlesACC(i),'XColor',[0.9725 0.9725 0.9725]);
    set(axesHandlesACC(i),'Color',[.15 .15 .15]);
    set(axesHandlesACC(i),'YLim', [-8 8]);
    set(axesHandlesACC(i),'YLimMode', 'manual');
    set(axesHandlesACC(i),'XLim', [0 2000/13.5]);
    set(axesHandlesACC(i),'XLimMode', 'manual');
    
    if(i > 12)
        xlabel(axesHandlesACC(i),'Samples');
    else
        set(axesHandlesACC(i), 'XTickLabel', '');
    end
    
    if(mod(i, 4) == 1)
        ylabel(axesHandlesACC(i),'g');
    else
        set(axesHandlesACC(i) ,'YTickLabel', '')
    end
    
    title(sprintf('ACC %i', i)) 

end

%%Open the COM interface, determine RATE

fopen(commObject);

pause(1);
fread(commObject,commObject.BytesAvailable);
fprintf(commObject, sprintf(['RATE 2000\r\n\r']));
pause(1);
fread(commObject,commObject.BytesAvailable);
fprintf(commObject, sprintf(['RATE?\r\n\r']));
pause(1);
data = fread(commObject,commObject.BytesAvailable);

emgRate = strtrim(char(data'));
if(strcmp(emgRate, '1925.926'))
    rateAdjustedEmgBytesToRead=1664;
else 
    rateAdjustedEmgBytesToRead=1728;
end


%% Setup interface object to read chunks of data
% Define a callback function to be executed when desired number of bytes
% are available in the input buffer
 bytesToReadEMG = rateAdjustedEmgBytesToRead;
 interfaceObjectEMG.BytesAvailableFcn = {@localReadAndPlotMultiplexedEMG,plotHandlesEMG,bytesToReadEMG};
 interfaceObjectEMG.BytesAvailableFcnMode = 'byte';
 interfaceObjectEMG.BytesAvailableFcnCount = bytesToReadEMG;
 
 bytesToReadACC = 384;
interfaceObjectACC.BytesAvailableFcn = {@localReadAnPlotMultiplexedACC, plotHandlesACC, bytesToReadACC};
interfaceObjectACC.BytesAvailableFcnMode = 'byte';
interfaceObjectACC.BytesAvailableFcnCount = bytesToReadACC;

% drawnow
% start(t);

% Open the interface object
try
    fopen(interfaceObjectEMG);
    fopen(interfaceObjectACC);
catch
    localCloseFigure(figureHandleACC,1 ,interfaceObjectACC, interfaceObjectEMG, commObject, t);
    delete(figureHandleEMG);
    error('CONNECTION ERROR: Please start the Delsys Trigno Control Application and try again');
end

%%
% Send the commands to start data streaming
fprintf(commObject, sprintf(['START\r\n\r']));
disp('run into the loop')
old=[];
sss=0;
count2=0;
total=[];START=0;   
all_combine=[];all_delta_wan_y=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%========================


IP_local  = '192.168.11.1'; 
port_remote = 5000;
IP_remote = '192.168.11.2'; 
port_local = 5000;
Role = 'server';

t_client = tcpip(IP_remote,port_remote,...
                'NetworkRole',Role,...
                'LocalPort',port_local,...
                'TimeOut',10,...
                'OutputBufferSize',8192);
            
            
t_client.OutputBuffersize=100000;
            
fopen(t_client);
disp('tcp_send is open!');
x_all = [];
y_all = [];all_9xyz=[];
global each_update_16 flag
direction=2.5; j=0;
while 1
   if ~isempty(flag)
       disp(flag);
      break; 
   end    
end










all_9xyz=zeros(300000,7);
from_EMG=1;to_EMG=0;
h = figure(100);


%% ================================

	%if the target is large, lower the resolution, we don't need that much
	%detail
	resize_image = (sqrt(prod(target_sz)) >= 100);  %diagonal size >= threshold
	if resize_image,
        disp(resize_image);
		pos = floor(pos / 2);
		target_sz = floor(target_sz / 2);
	end


	%window size, taking padding into account
	window_sz = floor(target_sz * (1 + padding));
	
% 	%we could choose a size that is a power of two, for better FFT
% 	%performance. in practice it is slower, due to the larger window size.
% 	window_sz = 2 .^ nextpow2(window_sz);

	
	%create regression labels, gaussian shaped, with a bandwidth
	%proportional to target size
	output_sigma = sqrt(prod(target_sz)) * output_sigma_factor / cell_size;
	yf = fft2(gaussian_shaped_labels(output_sigma, floor(window_sz / cell_size)));

	%store pre-computed cosine window
	cos_window = hann(size(yf,1)) * hann(size(yf,2))';	
	
	
% 	if show_visualization,  %create video interface
% 		update_visualization = show_video(img_files, video_path, resize_image);
% 	end

	%note: variables ending with 'f' are in the Fourier domain.
    
	time = 0;  %to calculate FPS
% 	positions = zeros(numel(img_files), 2);  %to calculate precision
    positions = [];
    INIT=0;
    frame = 0; loop_count = 0;
%             B= [ 0.0009446918438402,  0.00188938368768,0.0009446918438402
%             ];25
%             A= [1,   -1.911197067426,   0.9149758348014];
            
            B= [ 0.04125353724172,  0.08250707448344,  0.04125353724172 ];
            A= [1,   -1.348967745253,   0.5139818942197];
            
            
    all_dt=[];  loop=0;  init_current_all=[]; 
                
            y_EMG_afterfilter = [0 0 0 0 0 0 0;0 0 0 0 0 0 0;];
            x_EMG_now=[0 0 0 0 0 0 0;0 0 0 0 0 0 0;];
            all_intension=x_EMG_now;
    
    
    
    tic;
    last=toc;
    while ~isempty(snapshot(zed))
% 	while ~isempty(snapshot(zed)) && ~isempty(each_update_16)   
         loop=loop+1
        current_all=[];

        temp=each_update_16;
     
        for each =1:7
            if each == 3
                each = 10;
            end
            each_tongdao=temp(each:16:end);
            current_all=[current_all each_tongdao];
        end 
      
      
        
        if loop <= 10
            bias=[0 0 0 0 0 0 0];
        elseif INIT == 0
            bias=mean(all_9xyz(floor(end/2):end,:))
            sorted_all=sort(all_9xyz(floor(end/2):end,:))
            INIT = 1;
        end
        
        
        
        
         
        each_update_16=each_update_16(end-16+1:end);
%         all_9xyz=[all_9xyz; current_all;];
        to_EMG=to_EMG+size(current_all,1);
        all_9xyz(from_EMG:to_EMG,:)=current_all;
        from_EMG=from_EMG+size(current_all,1);
        
        intension=abs(mean(current_all)-bias);
        all_intension=[all_intension; intension;];

    x_EMG_now=[x_EMG_now(end-1:end,:); intension;];

    y_new=[0 0 0 0 0 0 0];
    for i = 1:length(B)
          y_new = y_new + (B(i) * x_EMG_now(end-i+1,:));
    end
    for i = 2:length(B)
          y_new = y_new - (A(i) * y_EMG_afterfilter(end-i+2,:));
    end
    y_EMG_afterfilter=[y_EMG_afterfilter; y_new;];
    
    
%     Hd=butter2_150;
%     after=filter(Hd,all_intension);
%     
%     figure;plot(all_intension(:,2));hold on; plot(y_EMG_afterfilter(:,2));hold on; plot(after(:,2));
%     legend('ori','first','second')
    
%     figure(91);plot(y_EMG_afterfilter(:,1)); hold on; plot(all_intension(:,1)); 
  
    
    
    
    
    
        direction= sin(loop/10 * 2 * pi); 

         x = 2*sin(i/1000 * 2 * pi);
         y = sin(i/1000 * 2 * pi);
         x_all = [x_all x];
         y_all = [y_all num2str(y)];
               
 %%
        
        
        send_buffer = [];
		%load image
        img = snapshot(zed);
        image_left = img(:, 1 : width/2, :);
        image_right = img(:,width/2 + 1:end,:);
        im = image_left;
		frame = frame + 1;
        loop_count = loop_count + 1;
        if(mod(loop_count,30) == 1)
            [~,center_left_image] = findTheBarrierInImage2(image_left);
            [~,center_right_image] = findTheBarrierInImage2(image_right);
            loop_count = 0;
        end
        
		if size(im,3) > 1,
			im = rgb2gray(im);
            im_right = rgb2gray(image_right);
		end
		if resize_image,
			im = imresize(im, 0.5);
            im_right = imresize(im_right,0.5);
		end

		tic()
		if frame > 1,
			%obtain a subwindow for detection at the position from last
			%frame, and convert to Fourier domain (its size is unchanged)
			patch = get_subwindow(im, pos, window_sz);
			zf = fft2(get_features(patch, features, cell_size, cos_window));
			
			%calculate response of the classifier at all shifts
			switch kernel.type
			case 'gaussian',
				kzf = gaussian_correlation(zf, model_xf, kernel.sigma);
			case 'polynomial',
				kzf = polynomial_correlation(zf, model_xf, kernel.poly_a, kernel.poly_b);
			case 'linear',
				kzf = linear_correlation(zf, model_xf);
			end
			response = real(ifft2(model_alphaf .* kzf));  %equation for fast detection

			%target location is at the maximum response. we must take into
			%account the fact that, if the target doesn't move, the peak
			%will appear at the top-left corner, not at the center (this is
			%discussed in the paper). the responses wrap around cyclically.
			[vert_delta, horiz_delta] = find(response == max(response(:)), 1);
			if vert_delta > size(zf,1) / 2,  %wrap around to negative half-space of vertical axis
				vert_delta = vert_delta - size(zf,1);
			end
			if horiz_delta > size(zf,2) / 2,  %same for horizontal axis
				horiz_delta = horiz_delta - size(zf,2);
			end
			pos = pos + cell_size * [vert_delta - 1, horiz_delta - 1];
		end

		%obtain a subwindow for training at newly estimated target position
		patch = get_subwindow(im, pos, window_sz);
		xf = fft2(get_features(patch, features, cell_size, cos_window));

		%Kernel Ridge Regression, calculate alphas (in Fourier domain)
		switch kernel.type
            case 'gaussian',
                kf = gaussian_correlation(xf, xf, kernel.sigma);
            case 'polynomial',
                kf = polynomial_correlation(xf, xf, kernel.poly_a, kernel.poly_b);
            case 'linear',
                
                kf = linear_correlation(xf, xf);
		end
		alphaf = yf ./ (kf + lambda);   %equation for fast training

		if frame == 1,  %first frame, train with a single image
			model_alphaf = alphaf;
			model_xf = xf;
		else
			%subsequent frames, interpolate model
			model_alphaf = (1 - interp_factor) * model_alphaf + interp_factor * alphaf;
			model_xf = (1 - interp_factor) * model_xf + interp_factor * xf;
		end

		%save position of left camera
		positions(frame,:) = pos; %  center points of the object in leftCamera [v u].
		        
        %templateMatching
        template_region = floor([pos([2,1]) - target_sz([2,1])/2, pos([2,1]) + target_sz([2,1])/2]);
        template = im(template_region(2):template_region(4),template_region(1):template_region(3));
        result = matchTemplateOCV(template, im_right(template_region(2):template_region(4),:)); % in order to lessen the calculation, 
        [~, idx] = max(abs(result(:)));
        [y, x] = ind2sub(size(result),idx(1));%  lefttop points of the object in rightCamera [v u].
        pos_right = [y + target_sz(1)/2 + template_region(2),x + target_sz(2)/2];%  center points of the object in rightCamera [v u].
        positions_right(frame,:) = pos_right;
        
        time = time + toc();
%         % 整理要发送的数据 send_buffer = [X_obj, Y_obj, Z_obj, N_barr, X_barr1, Y_barr1, Z_barr1]
        if(size(center_left_image,1) == size(center_right_image,1))
            [X_obj, Y_obj, Z_obj] = ProjectTo3D(pos(2)*2, pos(1)*2, pos_right(2)*2, pos_right(1)*2);
            send_buffer = [send_buffer, X_obj, Y_obj, Z_obj, double(size(center_left_image,1))];
            for i = 1:size(center_left_image,1)
                [X_barr, Y_barr, Z_barr] = ProjectTo3D(center_left_image(i,1),center_left_image(i,2),center_right_image(i,1),center_right_image(i,2));
                send_buffer = [send_buffer, X_barr, Y_barr, Z_barr];
            end        
        end
        disp(send_buffer); % =============================================================================
        fwrite(t_client,[88888.888,intension,direction,send_buffer],'double');%写入数字数据，每次发送360个double

        
% %         center_left_image
%         %visualization
%         box = [pos([2,1]) - target_sz([2,1])/2, target_sz([2,1])]; 
%         box_right = [pos_right([2,1]) - target_sz([2,1])/2, target_sz([2,1])]; 
%         h = figure(100);
%         subplot(1,2,1);
%         imshow(im);hold on;
%         rectangle('Position',box,'LineWidth',4,'EdgeColor','g');hold on;
%         plot(pos(2),pos(1),'ro');
%         for i = 1:size(center_left_image,1)
%             plot(center_left_image(i,1)./2,center_left_image(i,2)./2,'r*');hold on;
%         end
%         subplot(1,2,2);
%         imshow(im_right);hold on;
%         rectangle('Position',box_right,'LineWidth',4,'EdgeColor','g');  
%         plot(pos_right(2),pos_right(1),'ro');
%         for i = 1:size(center_right_image,1)
%             plot(center_right_image(i,1)./2,center_right_image(i,2)./2,'r*');hold on;
%         end
%         
        
        %if the key('p') is pressed, break out of the loop
        if strcmpi(get(h,'CurrentCharacter'),'p')
            disp('Transport is done!');
            fclose(t_client); 
            save all_9xyz.mat
            save y_EMG_afterfilter.mat
            save fuckall
            break; 
        end
        t0=toc;
        dt=-last+t0;
        last=t0;
        all_dt=[all_dt dt];
	end

	if resize_image,
		positions = positions * 2;
        positions_right = positions_right * 2;
	end
end



%% Implement the bytes available callback
%The localReadandPlotMultiplexed functions check the input buffers for the
%amount of available data, mod this amount to be a suitable multiple.

%Because of differences in sampling frequency between EMG and ACC data, the
%ratio of EMG samples to ACC samples is 13.5:1

%We use a ratio of 27:2 in order to keep a whole number of samples.  
%The EMG buffer is read in numbers of bytes that are divisible by 1728 by the
%formula (27 samples)*(4 bytes/sample)*(16 channels)
%The ACC buffer is read in numbers of bytes that are divisible by 384 by
%the formula (2 samples)*(4 bytes/sample)*(48 channels)
%Reading data in these amounts ensures that full packets are read.  The 
%size limits on the dataArray buffers is to ensure that there is always one second of
%data for all 16 sensors (EMG and ACC) in the dataArray buffers
function localReadAndPlotMultiplexedEMG(interfaceObjectEMG, ~,~,~, ~)
global rateAdjustedEmgBytesToRead;
bytesReady = interfaceObjectEMG.BytesAvailable;
bytesReady = bytesReady - mod(bytesReady, rateAdjustedEmgBytesToRead);%%1664

if (bytesReady == 0)
    return
end
% global data_arrayEMG
data = cast(fread(interfaceObjectEMG,bytesReady), 'uint8');
data = typecast(data, 'single');

global each_update_16 flag

global data_arrayEMG
flag = data;

if size(each_update_16,1) < rateAdjustedEmgBytesToRead*19
    each_update_16 = [each_update_16; data];
else
    each_update_16 = [each_update_16(size(data,1) + 1:size(each_update_16, 1));data];
end


if(size(data_arrayEMG, 1) < rateAdjustedEmgBytesToRead*19)
    data_arrayEMG = [data_arrayEMG; data];
else
    data_arrayEMG = [data_arrayEMG(size(data,1) + 1:size(data_arrayEMG, 1));data];
end
end


function localReadAnPlotMultiplexedACC(interfaceObjectACC, ~, ~, ~, ~)

bytesReady = interfaceObjectACC.BytesAvailable;
bytesReady = bytesReady - mod(bytesReady, 384);

if(bytesReady == 0)
    return
end
global data_arrayACC
data = cast(fread(interfaceObjectACC, bytesReady), 'uint8');
data = typecast(data, 'single');





if(size(data_arrayACC, 1) < 7296)
    data_arrayACC = [data_arrayACC; data];
else
    data_arrayACC = [data_arrayACC(size(data, 1) + 1:size(data_arrayACC, 1)); data];
end

end

%% Update the plots
%This timer callback function is called on every tick of the timer t.  It
%demuxes the dataArray buffers and assigns that channel to its respective
%plot.
function updatePlots(obj, Event,  tmp)
% global data_arrayEMG
% global plotHandlesEMG
% 
% for i = 1:size(plotHandlesEMG, 1) 
%     data_ch = data_arrayEMG(i:16:end);      
%     set(plotHandlesEMG(i), 'Ydata', data_ch)
% end
% 
% 
% global data_arrayACC
% global plotHandlesACC
% for i = 1:size(plotHandlesACC, 1)
%     data_ch = data_arrayACC(i:48:end);
%     set(plotHandlesACC(i), 'Ydata', data_ch)
% end
% drawnow

end


%% Implement the close figure callback
%This function is called whenever either figure is closed in order to close
%off all open connections.  It will close the EMG interface, ACC interface,
%commands interface, and timer object
function localCloseFigure(figureHandle,~,interfaceObject1, interfaceObject2, commObject, t)

%% 
% Clean up the network objects
if isvalid(interfaceObject1)
    fclose(interfaceObject1);
    delete(interfaceObject1);
    clear interfaceObject1;
end
if isvalid(interfaceObject2)
    fclose(interfaceObject2);
    delete(interfaceObject2);
    clear interfaceObject2;
end



if isvalid(t)
   stop(t);
   delete(t);
end

if isvalid(commObject)
    fclose(commObject);
    delete(commObject);
    clear commObject;
end

%% 
% Close the figure window
delete(figureHandle);
end

