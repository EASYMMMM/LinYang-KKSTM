function Hd = highpassfilter
%HIGHPASSFILTER 返回离散时间滤波器对象。

% MATLAB Code
% Generated by MATLAB(R) 9.9 and Signal Processing Toolbox 8.5.
% Generated on: 09-Aug-2021 15:57:47

% Butterworth Highpass filter designed using FDESIGN.HIGHPASS.

% All frequency values are in Hz.
Fs = 100;  % Sampling Frequency

Fstop = 1;           % Stopband Frequency
Fpass = 1.5;         % Passband Frequency
Astop = 80;          % Stopband Attenuation (dB)
Apass = 1;           % Passband Ripple (dB)
match = 'stopband';  % Band to match exactly

% Construct an FDESIGN object and call its BUTTER method.
h  = fdesign.highpass(Fstop, Fpass, Astop, Apass, Fs);
Hd = design(h, 'butter', 'MatchExactly', match);

% [EOF]