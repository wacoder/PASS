% *****************************************************************************************
% Proximity-detection-based Augmented RFID System (PASS)
%   based on Position Aware RFID Systems: The PARIS Simulation Framework
% *****************************************************************************************
% Sensatag Simulation
%
%
% ***** Copyright / License / Authors *****
% Copyright 2007, 2008, 2009, 2010, 2011 Daniel Arnitz
%   Signal Processing and Speech Communication Laboratory, Graz University of Technology, Austria
%   NXP Semiconductors Austria GmbH Styria, Gratkorn, Austria
% Copyright 2012 Daniel Arnitz
%   Reynolds Lab, Department of Electrical and Computer Engineering, Duke University, USA
% Copyright Jing Wang Identigo Lab
%
% This file is part of the PASS Simulation Framework.
%
% The PASS Simulation Framework is free software: you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software Foundation, either
% version 3 of the License, or (at your option) any later version.
%
% The PASS Simulation Framework is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License along with the PASS Simulation 
% Framework. If not, see <http://www.gnu.org/licenses/>.
%
% ********** Bahavior *********************
% output = sensatag_demodulation(signal, clock, settings)
%   Models the demodulator( demodulate signal according to settings and returns the sampled 
%   and demodulated binary signal in output)
%
%
% *****************************************************************************************

function output = sensatag_demodulation(signal, clock, settings)
version = 'beta 3.1';

% *****************************************************************************************
% version system

% just return version number
if nargin == 0
    output = version;
    return
end

% call version system
version_system();

% ******************************************************************************************
% internal settings

% downsampling for speed reasons ("turbo"): threshold for warnings
internalsettings.osr_min  =  8; % minimum sampling rate
internalsettings.rcatt_min = 30; % dB minimum attenuation of RC lowpass at new fs/2...?

% ******************************************************************************************
% input parameter checks / prepare input parameters
if nargin < 2
    criterr('No enough input arguments');
end

% check contents of settings
expected.name = 'settings';
expected.reqfields = {'fs','fc','fclk','agc_len', 'agc_ol', 'agc_nbins', 'tau', 'force_exact_rc',...
   'dvpeak', 'ar', 'af', 'slewrate', 'agc_sat', 'debouncelength', 'vdda', 'turbo', 'turbo_osfactor'};
% check
errortext = contentcheck(settings, expected);
% output
if ~isempty(errortext)
    err('Incomplete settings\n%s', errortext);
end

% empty signal
if isempty(signal) || var(signal) == 0
    critwarn('Length or variance of input signal is zero. Returning empty vector.')
    output = [];
    return
end

% -> column vectors
signal = signal(:);

% *********************************************************************************************
% load characteristic (lookup tables)

% load AGC amplitude characteristic
tagchar_demod = loadmat('tagchar_demodulator');

% *********************************************************************************************
% demodulator system: env(rectifier, AGC, RC-lowpass)
%   plus downsampling for performance reasons (if activated)

% downsampling factor (for performance reasons)
settings.dsf = round(settings.fs / settings.fclk / (settings.turbo_osfactor*2));

% get carrier peak amplitude for AGC
%   setup peakampl
settings.peakrms.fs      =   settings.fs; % sampling frequency in MHz
settings.peakrms.dsmode  =           ' '; % downsampling mode: no downsampling
settings.peakrms.dsf     =   settings.dsf; % downsmapling samples
settings.peakrms.nwin    =   round(settings.fs / settings.fc * settings.agc_len); % window length
settings.peakrms.ol      =   ceil(settings.peakrms.nwin * settings.agc_ol/100) / settings.peakrms.nwin * 100; % overlapping

% estimate peak
carrier_peak = sqrt(2) * peakrms(signal, settings.peakrms);

% rectifier
signal(find(signal < 0)) = 0; % half-wave rectifier

% RC lowpass, AGC => envelop
%   find closer index in RC filter coefficients
tagchar_demod.pos_fs = interp1(tagchar_demod.rc_fs, 1:tagchar_demod.settings.n_rc, settings.fs);
tagchar_demod.ind_fs = round(tagchar_demod.pos_fs);

%   create new RC filter coefficients if necessary (settings.tau=RC)
if isnan(tagchar_demod.pos_fs) || settings.tau ~= tagchar_demod.settings.rc_tau ||...
      ( settings.force_exact_rc && (tagchar_demod.ind_fs ~= tagchar_demod.pos_fs) )
   % make sure that the necessary toolbox is available (used only here => may not be available)
   if ~exist('tf', 'builtin')
      if isnan(tagchar_demod.pos_fs) || settings.tau ~= tagchar_demod.settings.tau
         err('Provided RC filter coefficients not applicable but cannot recalculate (toolbox missing).')
      else
         err('Cannot recalculate RC filter coefficients (toolbox missing). Consider switching off forced recalc.')
      end
   end
   % use c2d to discretize filter and finally tfdata to get filter coefficients
   [rc_b, rc_a] = tfdata(c2d(tf(1, [settings.tau, 1]), 1/settings.fs), 'v');
else
   % warn if there is no exact match
   if ~isnan(tagchar_demod.pos_fs) && (tagchar_demod.ind_fs ~= tagchar_demod.pos_fs)
      warn('No exact match for RC filter coefficients. Taking closest match (df=%.1f MHz).',...
         abs(settings.fs-tagchar_demod.rc_fs(tagchar_demod.ind_fs)));
   end
   rc_a = tagchar_demod.rc_a(tagchar_demod.ind_fs, :);
   rc_b = tagchar_demod.rc_b(tagchar_demod.ind_fs, :);
end
% filter
signal = filter(rc_b, rc_a, signal);

% downsampling for performance reasons
if settings.turbo
    % caculate attenuation of RC lowpass at oversampling frequency
    % osr*fclk
    rc_nyquistatt    = -20*log10(abs(freqz(rc_b, rc_a, [0, settings.fs/(2*settings.dsf)], settings.fs)));
    rc_nyquistatt(1) = [];
    % issue a message/warning
    msg('Downsampling by factor %i (new fs = %g x fclk), att of RC-lowp at new fs/2: %.1f dB.',...
       settings.dsf, settings.turbo_osfactor*2, rc_nyquistatt);
    if rc_nyquistatt < internalsettings.rcatt_min || settings.turbo_osfactor < internalsettings.osr_min
       critwarn('Low attenuation or critical oversampling factor.');
    end
    % downsample
    signal = signal(1 : settings.dsf : end);
    % adjust sampling clock
    clock = round(clock / settings.dsf);
    % make sure clock can still be used as index (...rounding)
    if clock(1) == 0
       clock(1) = 1;
    end
    if clock(end) > length(signal)
       clock(end) = length(signal);
    end
    % adjust settings
    settings.fs = settings.fs / settings.dsf;
    settings.ar = settings.ar * settings.dsf;
    settings.af = settings.af * settings.dsf;
end
    
% AGC (ideal)
if carrier_peak < tagchar_demod.vrf(1) || carrier_peak > tagchar_demod.vrf(end)
   % forced operability: keep demodulator functional under all circumstances
   if settings.agc_sat
      warn('Signal peak amplitude (%.4gV) outside characteristic (%.g-%.gV). Saturating.',...
         carrier_peak, tagchar_demod.vrf(1), tagchar_demod.vrf(end));
      carrier_peak = sanitize_setting('', carrier_peak, [tagchar_demod.vrf(1), tagchar_demod.vrf(end)]);
      % standard operations
   else
      warn('Signal peak amplitude (%.4gV) outside characteristic (%.g-%.gV). Returning empty vector.',...
         carrier_peak, tagchar_demod.vrf(1), tagchar_demod.vrf(end));
      output = []; return;
   end
end

% interpolate
[x, y] = meshgrid(tagchar_demod.vdda, tagchar_demod.vrf);
settings.agc_peak = interp2(x, y, tagchar_demod.venv, settings.vdda, carrier_peak);

% get signal peak amplitude (also for AGC)
%     setup peakampl
settings.peakrms.fs     =  settings.fs; % sampling frequency in MHz
settings.peakrms.dsmode =       'none'; % downsampling mode: random downsampling
settings.peakrms.nwin   = round(settings.fs / settings.fc * settings.agc_len); % window length ...
settings.peakrms.ol     = ceil(settings.peakrms.nwin * settings.agc_ol/100) / settings.peakrms.nwin * 100; % ... and overlapping
%     estimate (demodulated => ~DC)
demodulated_peak = peakrms(signal, settings.peakrms);

% scale to peak amplitude settings.agc_peak
signal = signal / demodulated_peak * settings.agc_peak;

% get the level
mid_level = mean(signal);
b_level = mean(signal(find(signal > mid_level)));
s_level = mean(signal(find(signal < mid_level)));
settings.dvpeak = (b_level - s_level)/2;


% *******************************************************************************************************
% demodulator system: vpeak, output sampling by fclk and debouncing

% prepare parameters
settings.slewrate = settings.slewrate / settings.fs; % V/s / Hz => V/sample
level = b_level- settings.dvpeak; % (assume carrier for t<0 => steady state already reached)

% generation of vpeak
vpeak = zeros(size(signal));
for i = 1 : length(signal) 
   % step to new level
   deltalevel = signal(i)-level-settings.dvpeak;
   % rise / fall speed
   if deltalevel > 0
      deltalevel = deltalevel * settings.ar;
   else
      deltalevel = deltalevel * settings.af;
   end
   % bounded slew rate
   if abs(deltalevel) > abs(settings.slewrate)
         deltalevel = sign(deltalevel) * settings.slewrate;
   end
   % new level
   level = level + deltalevel;
   % saturate
   if level > settings.vdda
      level = settings.vdda;
   end
   if level < 0 % vgnd
      level = 0;
   end
   % and store
   vpeak(i) = level;
end

output.signal = signal;
output.level = vpeak;

% comparator and sampling
output.demod = double( signal(clock) > vpeak(clock) );

% debouncing (filter shifts all edges, so this should not be a problem for timing detection)
debouncefilter = ones(settings.debouncelength,1) / settings.debouncelength;
output.demod = round(filter(debouncefilter, 1, output.demod));


return







