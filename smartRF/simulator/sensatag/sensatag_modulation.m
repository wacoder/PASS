%*********************************************************************************************
% Proximity-detection-based Augmented RFID System (PASS)
%   based on Position Aware RFID Systems: The PARIS Simulation Framework
%*********************************************************************************************
%
% Senstage Simulation
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
% ********** Behavior ***************
% Verison = sensatag_modulation()
%
% settings = sensatag_modulation(data, settings)
% Returns completed( e.g. the needed carrier length to encode and modulate DATA according to 
% SETTINGS)
%*********************************************************************************************
function [modcarrier, internal] = sensatag_modulation(varargin)
version = 'beta 3.1';

if nargin == 0
    modcarrier = version;
    return
end

version_system();


% ********************************************************************************************
% internal settings

% filename of filterbank channel coefficients
internalsettings.fbcharfile = 'tagchar_modulator_fb';

% maximum filterbank length as fraction of min length("high", "low")
internalsettings.nfft_maxfraction = 5; % (< 2 is not a good idea)

% tolerances before a warning is issued
internalsettings.lf_tol  =  100; % Hz "fclk multiple of LF"
internalsettings.imagtol = 1e-4; % im/re after synthesis

% factor for maximum error created by finite filterbank resolution
% (approximate, cf. output of plots_tag_modulation.m) 
internalsettings.fbdiff = 1/3; % times diff(spectrum)


% *******************************************************************************************************
% input parameter checks

% number of input parameters
if nargin == 2
   justreturnsettings = true;
   data     = varargin{1}(:);
   settings = varargin{2};
elseif nargin == 3
   critwarn('External modulation signal. Setting fclk to fs.');
   justreturnsettings = false;
   carrier        = varargin{1}(:);
   modsignal_fclk = varargin{2}(:);
   settings       = varargin{3};
   settings.fclk  = settings.fs; % fclk==fs (fastest possible reaction)
   clock          = 1:length(modsignal_fclk);
   data           = [0]; % dummy
elseif nargin == 4
   justreturnsettings = false;
   carrier  = varargin{1}(:);
   data     = varargin{2}(:);
   clock    = varargin{3}(:);
   settings = varargin{4};
else
   criterr('Wrong amount of input arguments.');
end

% check contents of settings
%     prepare required data
expected.name = 'settings';
expected.reqfields = {'fs', 'fc', 'fclk', 'lf', 't0', 'charfile', 'nfft', 'nfilt',...
   'pfnc_bias'};
%     check
errortext = contentcheck(settings, expected);
%     output
if ~isempty(errortext)
   err('Incomplete settings\n%s', errortext);
end

% length of carrier signal checked below (when timing is fixed)

% check if data signal has alphabet {0,1}
if sum(data == 0) + sum(data == 1) ~= length(data)
   err('Data has non-binary alphabet. Only binary signals with alphabet {0,1} allowed.')
end

% for external modulation signal: check if modsignal_fclk has alphabet {0,1}
if nargin == 3 && ( sum(modsignal_fclk == 0) + sum(modsignal_fclk == 1) ~= length(modsignal_fclk) )
   err('External modulation signal must have binary alphabet ({0,1}).')
end

% check settings.nfft (power of 2)
if log2(settings.nfft) ~= round(log2(settings.nfft))
   err('Filterbank FFT size (settings.nfft) has to be a power of 2.');
end

% mode?
%     filter (transmit signal)
if nargin == 4 && isempty(data) && isempty(clock)
   settings.mode = 'filter';
%     just reflect
elseif nargin == 4 && sum(data)==0 && isempty(clock)
   settings.mode = 'modulate';
   clock = [1, length(carrier)];
%     modulate data
else
   settings.mode = 'modulate';
end
% initialize struct internal
%     power levels
internal.pav = NaN;
% comment Septs. 23rd, 2013 jingwang
% internal.pin = NaN;
% internal.pic = NaN;
% %     power supply voltage
% internal.vdda = NaN;
% %     functional
% internal.func =  0;
% comment Septs.23rd, 2013 jingwang

%     frequency resolution of filterbank
internal.fres = NaN;
%     maximum error caused by filterbank not including interpolation [abs, angle]
internal.maxerr_mod = [NaN, NaN];
internal.maxerr_umd = [NaN, NaN];
%     linear model
internal.rho  = NaN;
internal.drho = NaN;
%     impedances
internal.zic = NaN;
internal.zat = NaN;


% *******************************************************************************************************
% lengths (encoding takes care of data rates; here we are at subcarrier level; Smod = 0/1)

% modulation
if nargin ~= 3 && strcmpi(settings.mode, 'modulate')
   % check if data is empty (...likely a mistake)
   if isempty(data)
      critwarn('Data vector is empty.');
   end
   
   % at clock frequency fclk (rounding: fclk has to be high enough to satisfy LF tolerances) [???]
   %     issue warning if fclk is not a multiple of LF
   if strcmpi(settings.mode, 'modulate') && mod(settings.fclk, settings.lf) > internalsettings.lf_tol
      critwarn('Tag clock frequency is not a multiple of LF (original: %.1f kHz, corrected: %.1f kHz)', ...
         settings.lf, settings.fclk / round(settings.fclk/settings.lf));
   end
   %     subcarrier period
   settings.t_fclk_s = round(settings.fclk / settings.lf);
   %     length of modulated (0) and unmodulated (1) halfbit (make modulated halfbit always shorter -> energy) 
   length0_fclk_s = floor(settings.t_fclk_s / 2);
   length1_fclk_s = ceil(settings.t_fclk_s / 2);
   %     length of modulated part
   settings.length_fclk_s = (length(data)-sum(data)) * length0_fclk_s + sum(data)*length1_fclk_s;  % Smod==0 + Smod==1

   % at sampling frequency fs (rounding error here: don't care)
   %     length of modulated (0) and unmodulated (1) halfbits
   length0_s = round(length0_fclk_s * settings.fs / settings.fclk);
   length1_s = round(length1_fclk_s * settings.fs / settings.fclk);   
   
   % modulation with external modulation signal: no checks
elseif nargin == 3 && strcmpi(settings.mode, 'modulate')
   length0_s = length(carrier);
   length1_s = length(carrier);
   
% no modulation, just filter ... like modulating a zero that lasts for the whole carrier length
else
   % "the whole carrier lenght"
   clock = [1, length(carrier)];
   % before (and after) data, there are zeros
   data  = []; 
   % set t0 to zero (no unfiltered parts!)
   settings.t0 = 0;
   % no modulation patterns
   length0_fclk_s = 0;
   length1_fclk_s = 0;
   settings.length_fclk_s  = 0;
end

% load filterbank characteristics
tagchar_mod_filterbank = loadmat(internalsettings.fbcharfile);

% check if filter is longer than a fraction of backscatter link period
if strcmpi(settings.mode, 'modulate') && (settings.nfft > min(length0_s, length1_s)/internalsettings.nfft_maxfraction)
   nfft = 2^floor(log2(min(length0_s, length1_s)/internalsettings.nfft_maxfraction));
   critwarn('Filterbank too large for LF. Reducing channels to %i (old: %i).', nfft, settings.nfft);
   settings.nfft = nfft;
end

% length of "leadin" and "leadout"
%     length of t0 in samples
settings.t0_s = round( settings.t0 * settings.fs );
%     make sure modulated part is not truncated in case of clock phase shift and clock jitter
%     ... one tclk for phase shift, one tclk for jitter (on safe side) plus nfft/2
if strcmpi(settings.mode, 'modulate') && (settings.t0_s < 2*round(settings.fs / settings.fclk) + settings.nfft/2)
   critwarn('Saturating t0_s to minimum to prevent truncation of mod part.')
   settings.t0_s = 2*round(settings.fs / settings.fclk) + settings.nfft/2;
end
%     in seconds (... sanitized setting)
settings.t0 = settings.t0_s / settings.fs;

% group delay of filterbank
%     delay for carrier
settings.grpdel_s = settings.nfft/2 * ( settings.nfilt - 1);
%     delay for modulation signal
%        the modulation signal has a smaller delay than the carrier (grpdel_s); nontheless grpdel_s is
%        removed from the modulated signal => this has to be compensated by adding the delay difference
%         to the clock signal (plus block processing!)
%           ... modulation reaches ind_clk == 1 at round(t0_s/(nfft/2)) if set delay is settings.t0_s+
%               round(settings.fs/settings.fclk)-settings.nfft/4 iand initial clock jitter is 
%               compensated: -clock(1)+1
%            ... shift delay in that case is below settings.t0_s (removal of grpdel_s)
%            ... error to removed grpdel_s measured to 50% value of first transient is approximately
%                (settings.nfilt/2-2)*settings.nfft/2
%           => add delay for ind_clk==1@i*nfft/2==t0_s, add removed grpdel_s and compensate
%           ... grpdel_s - (nfilt/2-2)*nfft/2 = (nfilt/2+1)*nfft/2
%     ... problem: t0_s = 0 is dangerous (not "unmodulated" at beginning)
settings.moddel_s = round(settings.fs/settings.fclk) - settings.nfft/4 +... % ind_clk==1@i*nfft/2==t0_s
                    (settings.nfilt/2+1)*settings.nfft/2; % difference of real delay to rem. grpdel_s
                 
% just return settings if requested
if justreturnsettings
   % length of whole modulated part (estimate; will change because of clock jitter)
   settings.length_s = (length(data)-sum(data)) * length0_s + sum(data)*length1_s +... % Smod==0 + Smod==1
      2*settings.t0_s +... % additional (manual) leadin/leadout
      settings.grpdel_s; % group delay of filterbank (deleted part)

   %     make settings.lenths_s a multiple of settings.nfft/2 (for PPN filterbank)
   settings.length_s = settings.length_s + settings.nfft/2 - mod(settings.length_s, settings.nfft/2); % samples
   settings.length   = settings.length_s / settings.fs; % s

   % return these settings
   modcarrier = settings;
   return
end
  
% recalculate length @ fs (... clock jitter of tag clock)
% (Attention: length_s is length of modulated/filtered part and not carrier length!)
settings.length_s = clock(end); % samples
settings.length   = settings.length_s / settings.fs; % s

% check length of carrier signal now that timing is fixed (group delay is already included in settings.length_s)
%     make sure carrier length is a multiple of settings.nfft
if mod(length(carrier), settings.nfft/2) ~= 0
   critwarn('Length of carrier (%i) is no multiple of settings.nfft/2 (%i). Truncating.', length(carrier), settings.nfft/2);
   carrier(end-mod(length(carrier), settings.nfft/2)+1:end) = [];
end
%     check carrier length
if strcmpi(settings.mode, 'modulate') && (length(carrier) < settings.length_s)
   err('Insufficient length of carrier signal for given data vector (%i < %i).', length(carrier), settings.length_s);
end


% *******************************************************************************************************
% create modulation signal @ fclk (in range 0...1)

% build modulation signal at tag-clock level (only if generated internally)
if nargin ~= 3
   mod0_fclk = zeros(length0_fclk_s, 1);
   mod1_fclk = ones(length1_fclk_s, 1);
   modsignal_fclk = zeros(settings.length_fclk_s, 1);
   index = 1;
   %     create signal bit per bit
   for i = 1 : length(data)
      if data(i) == 0
         modsignal_fclk(index:index+length0_fclk_s-1) = mod0_fclk;
         index = index + length0_fclk_s;
      else
         modsignal_fclk(index:index+length1_fclk_s-1) = mod1_fclk;
         index = index + length1_fclk_s;
      end
   end
end


% *******************************************************************************************************
% characteristics: reflection coefficients of tag input + determine power levels and support

% load reflection coefficient characteristic
tagchar_mod = loadmat(settings.charfile);

% determine incident (available) carrier power level

   %     setup estimator (best estimation for carrier frequency)
   %     ... window/overlapping size multiples of carrier period 1/f0, ~50% overlapping
   %     ... window size max 1/4 t_lf samples (for time-variant: rho can be selected 4 times per t_lf)
   %         (if applicable)
   setup_est.f0 = settings.fc / settings.fs;
   setup_est.n  = round(ceil( settings.fc/(4*settings.lf) ) / setup_est.f0 );
   setup_est.ol = round(ceil( setup_est.n/2 * setup_est.f0 ) / setup_est.f0) / setup_est.n * 100;
   %     estimate (for now: only time-invariant, i.e. short-time stationary channel)
   %     ... average power => assumes that the current signal is representative and tag is well buffered
   tagchar_mod.pav_hat = mean(est_power(carrier, setup_est));

%     output
msg('Carrier power level (variance): Pav = %.2e uW (%.2f dBm)', tagchar_mod.pav_hat*1e6, 10*log10(tagchar_mod.pav_hat*1e3));

% get operating point (combination power/frequency: position, rounded index and vicinity)
%     available power
tagchar_mod.pos_pav = interp1(tagchar_mod.pav, [1:1:tagchar_mod.settings.np], tagchar_mod.pav_hat, 'linear',  NaN);
tagchar_mod.ind_pav = round(tagchar_mod.pos_pav);
if floor(tagchar_mod.pos_pav) == tagchar_mod.settings.np % very unlikely, but possible
   tagchar_mod.vic_pav = floor(tagchar_mod.pos_pav) + [0,0];
else % standard case
   tagchar_mod.vic_pav = floor(tagchar_mod.pos_pav) + [0,1];
end 
%     frequency (non-extrapolated)
tagchar_mod.pos_fch = interp1(tagchar_mod.fch, [1:1:tagchar_mod.settings.nfch], settings.fc, 'linear', NaN);
tagchar_mod.ind_fch = round(tagchar_mod.pos_fch);
if floor(tagchar_mod.pos_fch) == tagchar_mod.settings.nfch % very unlikely, but possible
   tagchar_mod.vic_fch = floor(tagchar_mod.pos_fch) + [0,0];
else % standard case
   tagchar_mod.vic_fch = floor(tagchar_mod.pos_fch) + [0,1];
end

% if frequency is outside characteristic: setup problem
if isnan(tagchar_mod.pos_fch)
   err('Carrier frequency (settings.fc) out of characteristic range (%.2f to %.2f MHz). Check setup.',...
   tagchar_mod.fch(1)/1e6, tagchar_mod.fch(end)/1e6);
end

% do we have characteristic support (if power in range of pav vector and no NaNs in vicinity of op.pt.) ?
%     extrapolated (if not: modulation/reflection will not work at all)
tagchar_mod.support.ext = ~isnan(tagchar_mod.pos_pav) && ...
   ~any(isnan(sum(sum(tagchar_mod.rhoext_pav(1+tagchar_mod.settings.nf_out1+tagchar_mod.vic_fch, :, tagchar_mod.vic_pav)))));
%     non-extrapolated (if not: tag is definitely not functional)
%     ... the nonextrap. char. contains values far below pmin => this cannot be covered by the pmin check
%         below
tagchar_mod.support.nonext = ~isnan(tagchar_mod.pos_pav) && ...
   ~any(isnan(sum(sum(tagchar_mod.rho_pav(tagchar_mod.vic_fch, :, tagchar_mod.vic_pav)))));

% calculate input power Pin and chip input power Pic
% ... note that tagchar_mod.p_in might be NaN for high power levels, even if there is support (interp1)
if tagchar_mod.support.ext % we need at least extrapolated support to do this
   % we are doing a square modulation (below); get input power for modulated and unmodulated state
   [x, y] = meshgrid(tagchar_mod.vic_pav, tagchar_mod.vic_fch);
   %     reflection coefficient unmodulated/modulated around operating point
   rho0 = interp2c(x,y, squeeze(tagchar_mod.rhoext_pav(1+tagchar_mod.settings.nf_out1+tagchar_mod.vic_fch,...
      end, tagchar_mod.vic_pav)), tagchar_mod.pos_pav, tagchar_mod.pos_fch, 'linear', NaN);
   rho1 = interp2c(x,y, squeeze(tagchar_mod.rhoext_pav(1+tagchar_mod.settings.nf_out1+tagchar_mod.vic_fch,...
        1, tagchar_mod.vic_pav)), tagchar_mod.pos_pav, tagchar_mod.pos_fch, 'linear', NaN);
   % reflection power for unmodulated/modulated state 
   % Sept. 23rd, Jing Wang, Identigo
   p_rf0 = tagchar_mod.pav_hat*abs(rho0)^2;
   p_rf1 = tagchar_mod.pav_hat*abs(rho1)^2;
   prf_hat = p_rf1*mean(modsignal_fclk) + p_rf0*(1-mean(modsignal_fclk));
   
   % output
   msg('Reflection power level :%.2e uW (%.2e dbm), Unmodulated, Prf0 = %.2e uW (%.2f dbm); Modulated, Prf1 = %.2e uW (%.2f dbm)',...
       prf_hat,10*log10(prf_hat),p_rf0, 10*log10(p_rf0), p_rf1, 10*log10(p_rf1));
   
% comment Sept. 23rd 2013    
%     %     input power for unmodulated/modulated
%    p_in0 = tagchar_mod.pav_hat * ( 1 - abs(rho0)^2 );
%    p_in1 = tagchar_mod.pav_hat * ( 1 - abs(rho1)^2 );
%    %     position for chip input power (unmodulated/modulated; 2Dinterp approximated by 2x1D interp)
%    pos_pic0(1) = interp1(squeeze(tagchar_mod.pin(tagchar_mod.vic_fch(1), end, :)), 1:tagchar_mod.settings.np, p_in0, 'linear');
%    pos_pic0(2) = interp1(squeeze(tagchar_mod.pin(tagchar_mod.vic_fch(2), end, :)), 1:tagchar_mod.settings.np, p_in0, 'linear');
%    pos_pic1(1) = interp1(squeeze(tagchar_mod.pin(tagchar_mod.vic_fch(1),   1, :)), 1:tagchar_mod.settings.np, p_in1, 'linear');
%    pos_pic1(2) = interp1(squeeze(tagchar_mod.pin(tagchar_mod.vic_fch(2),   1, :)), 1:tagchar_mod.settings.np, p_in1, 'linear');  
%    pos_pic0 = pos_pic0(2) * mod(tagchar_mod.pos_fch,1) + pos_pic0(1) * (1-mod(tagchar_mod.pos_fch,1));
%    pos_pic1 = pos_pic1(2) * mod(tagchar_mod.pos_fch,1) + pos_pic1(1) * (1-mod(tagchar_mod.pos_fch,1));
%    %     chip input power for unmodulated/modulated   
%    p_ic0 = interp1(1:tagchar_mod.settings.np, tagchar_mod.pic, pos_pic0, 'linear');
%    p_ic1 = interp1(1:tagchar_mod.settings.np, tagchar_mod.pic, pos_pic1, 'linear');
%    %     average powers taking modulation duty cycle into account (50% for EPC class-1 gen-2)
%    if isempty(modsignal_fclk)
%       tagchar_mod.pin_hat = p_in0;
%       tagchar_mod.pic_hat = p_ic0;
%    else
%       % combine power, taking modulation into account
%       pin_hat = p_in1 * mean(modsignal_fclk) + p_in0 * (1-mean(modsignal_fclk));
%       pic_hat = p_ic1 * mean(modsignal_fclk) + p_ic0 * (1-mean(modsignal_fclk));
%       % bias towards unmod to take the power supply buffer into account (until buffer is implemented)
%       % 0..1, 0: mod/unmod (continuous operation), 1: unmod only (short term)
%       % (for pin and pic to avoid pic > pin)
%       tagchar_mod.pin_hat = p_in0 * settings.pfnc_bias + pin_hat * (1-settings.pfnc_bias);
%       tagchar_mod.pic_hat = p_ic0 * settings.pfnc_bias + pic_hat * (1-settings.pfnc_bias);
%    end
% else
%    tagchar_mod.pin_hat = NaN; % don't know
%    tagchar_mod.pic_hat = NaN;
% end
% %     output
% msg('Input power / Chip input power: Pin = %.2e uW (%.2f dBm) / Pic = %.2e uW (%.2f dBm)',...
%    tagchar_mod.pin_hat*1e6, 10*log10(tagchar_mod.pin_hat)+30, tagchar_mod.pic_hat*1e6, 10*log10(tagchar_mod.pic_hat)+30);
% comment Sept. 23rd, 2013

%     variable cleanup
clear('rho0','rho1','p_rf0','p_rf1','prf_hat');

% Sensatag has battery, Power to IC is not needed.
tagchar_mod.ind_fc = interp1(tagchar_mod.fch, 1:tagchar_mod.settings.nfch, settings.fc, 'nearest');

% comment Sept. 23rd, 2013
% % comment on Sept. 23rd, Jing Wang
% tagchar_mod.support.functional = tagchar_mod.support.ext && tagchar_mod.support.nonext &&...
%    ~isnan(tagchar_mod.pic_hat) && tagchar_mod.picopmin(tagchar_mod.ind_fc) <= tagchar_mod.pic_hat;
% % comment on Sept. 23rd, Jing Wang
% 
% % calculate power supply voltage (Vdda)
% tagchar_pwr = loadmat(settings.pwrcharfile);
% if ~isfield(tagchar_pwr, lower(settings.pwrchar))
%    err('Unsupported power supply characteristic "%s".', settings.pwrchar);
% else
%    vdda = interp1(tagchar_pwr.pic, tagchar_pwr.(lower(settings.pwrchar)), 10*log10(tagchar_mod.pic_hat)+30, 'nearest', 'extrap');
% end
% %     output
% msg('Power supply voltage: Vdda = %.2f V (saturated to bounds)', vdda);
% comment Sept.23rd, 2013

% return some internal values
%     power levels
internal.pav = tagchar_mod.pav_hat;

% comment Sept. 23rd jingwang
% internal.pin = tagchar_mod.pin_hat;
% internal.pic = tagchar_mod.pic_hat;
% %     functional
% internal.func = tagchar_mod.support.functional;
% %     power supply voltage
% internal.vdda = vdda;
% comments Setp. 23rd jingwang

% decide what to do next
%     absolutely no characteristic support => modulation/reflection will not work
if ~tagchar_mod.support.ext
   msg('No characteristic support for this combination Pav,fc. Returning a zero vector.');
   modcarrier = zeros(length(carrier) - settings.grpdel_s, 1);
   return
end

% comment Sept. 23rd jingwang
% %     tag is not functional
% if ~tagchar_mod.support.functional
%    msg('Tag is not functional for this combination incident power / carrier frequency.');
%    % forced modulation/reflection (we have at least extrapolated support)
%    if settings.force
%       critwarn('Forced modulation/reflection.');      
%    % modulation: switch to unmodulated reflection
%    elseif strcmpi(settings.mode, 'modulate') && ~settings.force
%       msg('Switching from modulated to unmodulated reflection.');
%       modsignal_fclk = modsignal_fclk * 0;
%    % filter (transmission): tag not functional and not forced => no point in calculating transmitted signal
%    elseif strcmpi(settings.mode, 'filter') && ~settings.force
%       modcarrier = zeros(length(carrier) - settings.grpdel_s, 1);
%       return
%    % to be on the safe side
%    else
%       criterr('Ended up in an undefined state (not functional, mode=%s, force=%i).', settings.mode, settings.force);
%    end
% comments Sept. 23rd jingwang

end
%     only extrapolated support (this is only important if we continued)
if ~tagchar_mod.support.nonext
   critwarn('Only extrapolated characteristic support for this combination Pav,fc; error may be arbitrary.');
end

% output support
%     determine support boundaries
%     ... identical for all rmod because pav is calulated without modulation  => implicit assumption that
%         chip power supply is stable during modulation periods and those periods are kept short)
%     ... start/end with "no support" (1-0.1=0.9); bias towards non-NaN (-0.1); 
tagchar_mod.support.range = find(~isnan(sum(tagchar_mod.rho_pav(:, end, tagchar_mod.vic_pav), 3)));
tagchar_mod.support.bound = findzeros([0.9; isnan(sum(tagchar_mod.rho_pav(:, end, tagchar_mod.vic_pav), 3)) - 0.1; 0.9]) - 1;
%     safety check: length tagchar_mod.support.bound has to be 2,4,6,... (...no-yes-no-yes-no...)
if mod(length(tagchar_mod.support.bound), 2) ~= 0
   criterr('Characteristic support identification failed.');
end
%     output support
msg('Characteristic support:')
msg('   Extrapolated: %.1f to %.1f MHz', tagchar_mod.f(1)/1e6, tagchar_mod.f(end)/1e6)
if ~isempty(tagchar_mod.support.bound)
   mtext = sprintf('%.1f to %.1f MHz and ',...
      tagchar_mod.fch(tagchar_mod.support.bound(1:end/2))/1e6, tagchar_mod.fch(tagchar_mod.support.bound(end/2+1:end))/1e6); 
   msg('   Non-Extrapolated: %s', mtext(1:end-4));
end


% *******************************************************************************************************
% filter to obtain backscattered / transmitted signal

% check if provided channel filter coefficients are applicable, recalculate if necessary
if (settings.nfft == tagchar_mod_filterbank.settings.nfft) && (settings.nfilt == tagchar_mod_filterbank.settings.nfilt)
   coeff = tagchar_mod_filterbank.coeff;
else 
   msg('Provided filterbank channel filter coefficients not applicable; recalculating.');
   coeff = npr_coeff(settings.nfft, settings.nfilt);
end

% warning if settings.fs exceeds sampling frequency of filterbank (extrapolation has to be done)
if settings.fs * (settings.nfft-1)/(2*settings.nfft) > tagchar_mod.f(end)
   critwarn('Sampling frequency exceeds fs of characteristic, extrapolating with rho_max=%f.',...
      tagchar_mod.settings.rhomax);
end

% create overall filter impulse response w. homogenous resolution for filterbank
%     homogenous frequency resolution, unmod and mod
%     ... spline interpolation is not a good idea here, because it leads to |rho|>1 if the 
%         characteristic has been truncated to |rho|~1 (high steepness plus saturation)
[x , y ] = meshgrid(tagchar_mod.pav(tagchar_mod.vic_pav), tagchar_mod.f);
[xi, yi] = meshgrid(tagchar_mod.pav_hat, linspace(0,settings.fs/2,settings.nfft/2+1));
hh0 = interp2c(x,y, squeeze(tagchar_mod.rhoext_pav(:, end, tagchar_mod.vic_pav)), xi, yi,...
	'linear', tagchar_mod.settings.rhomax);
hh1 = interp2c(x,y, squeeze(tagchar_mod.rhoext_pav(:,   1, tagchar_mod.vic_pav)), xi, yi,...
	'linear', tagchar_mod.settings.rhomax);
%     check
if any(abs(hh0) > 1) || any(abs(hh1) > 1)
   critwarn('Resampling of characteristic failed (|rho|>1): saturating. Check sampling rate and interpolation.');
   ind0 = find(abs(hh0) > 1);
   ind1 = find(abs(hh1) > 1);
   hh0(ind0) = hh0(ind0) / abs(hh0(ind0));
   hh1(ind1) = hh1(ind1) / abs(hh1(ind1));
end

% change to transmit signal for mode filter
if strcmpi(settings.mode, 'filter')
   warn('Phase not supported in filter mode (phase of returned signal identical to carrier phase).')
   hh0 = sqrt(1 - abs(hh0).^2);
   hh1 = hh0; % just to make sure (should be unmod anyway)
end

% calculate some values
%     frequency resolution of filterbank
internal.fres = settings.fs / settings.nfft; % Hz
%     maximum error caused by filterbank not including interpolation [abs, angle]
%     ... 1/fbdiff*maximum difference between bins (cf. frequency sweep as created by plots_tag_modulation.m)
fch_range_i = [1+tagchar_mod.settings.nf_out1, tagchar_mod.settings.nf_out1 + tagchar_mod.settings.nfch];
internal.maxerr_mod = [...
   max(abs(diff(  abs(hh1(fch_range_i(1):fch_range_i(2))))))*internalsettings.fbdiff,...
   max(abs(diff(angle(hh1(fch_range_i(1):fch_range_i(2))))))*internalsettings.fbdiff*180/pi];
internal.maxerr_umd = [...
   max(abs(diff(  abs(hh0(fch_range_i(1):fch_range_i(2))))))*internalsettings.fbdiff,...
   max(abs(diff(angle(hh0(fch_range_i(1):fch_range_i(2))))))*internalsettings.fbdiff*180/pi];
%     linear model
internal.rho  = (hh1 + hh0) / 2;
internal.drho = internal.rho - hh0;

% output
%     frequency resolution
msg('Resolution of filterbank: %.3f MHz / %.3f us', internal.fres/1e6, 1e6/internal.fres);
%     and maximum error 
msg('Approximate max error for frequ between bins (highres area of tagchar, not including interpolation):')
msg('    mod: %.3e  %.3e deg', internal.maxerr_mod);
msg('   umod: %.3e  %.3e deg', internal.maxerr_umd);

% include negative frequencies (-> for npr_... functions)
% ... Note that the original npr_analysis used the IFFT, not the FFT (=> conj(hh0), conj(hh1)). The
%     modified version used here uses an FFT for the analysis (=> no modifications to hh0 and hh1 
%     necessary).
hh0 = [hh0(1:end); conj(hh0(end-1:-1:2))];
hh1 = [hh1(1:end); conj(hh1(end-1:-1:2))];

% analysis filterbank
spectrum = npr_analysis(coeff, carrier);

% free some memory (carrier needs a lot of RAM + filter will take some time)
clear('carrier');

% add initial delay to clock and compensate for modulation signal delay
clock = clock + settings.t0_s + settings.moddel_s;

% filter
%     generate index list (spectra are 50% overlapped; clock has jitter)
ind_clk = 1 + floor(interp1(clock, [1:1:length(clock)], [1:size(spectrum, 2)]*settings.nfft/2, 'linear', 'extrap'));
%     filter loop
for i = 1 : size(spectrum, 2)  
   % for t < 0 || t > modulation length: not modulated
   if (ind_clk(i) < 1) || (ind_clk(i) > length(modsignal_fclk)) || (modsignal_fclk(ind_clk(i)) == 0)
      spectrum(:, i) = spectrum(:, i) .* hh0; % not modulated
   else
      spectrum(:, i) = spectrum(:, i) .* hh1; % modulated
   end
end

% synthesis filterbank
modcarrier = npr_synthesis(coeff, spectrum);

% check imaginary part (has to be close to zero)
% ... imaginary part may be rather high if hh0 approx conj(hh1)
if mean(imag(modcarrier(settings.grpdel_s+1:end)).^2) / mean(real(modcarrier(settings.grpdel_s+1:end)).^2) > internalsettings.imagtol
   critwarn('Backscatter signal has considerable imaginary part (avg im/re = %.2g). Check filter operation for symmetry!', mean(imag(modcarrier).^2)/mean(real(modcarrier).^2));
end

% make real and remove transient (group delay); make modcarrier a column vector
modcarrier = real(modcarrier(settings.grpdel_s+1:end)).';

% multiply by abs(input impedance @ operating point) to obtain demodulator input signal in 'filter' mode
% ... reflected signal can be seen as [Vrms]@R=1Ohm (effective power: P=var(modcarrier)=U^2/R)
% ... use Zat (and not alternative) to avoid accumulation of errors (we're using mostly estimates!)
%     alternative: remove 1/Zat and add Pin/Pic
if strcmpi(settings.mode, 'filter')
   % get impedances at operating point
   %     chip input impedance
%    [x , y ] = meshgrid(tagchar_mod.pic, tagchar_mod.fch);
%    [xi, yi] = meshgrid(tagchar_mod.pic_hat, settings.fc);
   
%    internal.zic = interp2c(x,y, tagchar_mod.zic, xi, yi, 'linear', NaN);
   internal.zic  =  7.0085e+01 - 1.1136e+02i;
   %     assembly impedance
   internal.zat = complex(tagchar_mod.settings.rat, -1/(2*pi*settings.fc*tagchar_mod.settings.cat));
   % multiply => peak voltage on chip input impedance (no phase!)
   modcarrier = modcarrier / sqrt(real( 1/internal.zic + 1/tagchar_mod.zmod(end) + 1/internal.zat ));
end


return



