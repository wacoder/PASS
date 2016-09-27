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
% version = sensatag_decoding()
%   just return the verison number
% [data, linkinfo] = sensatag_decoding(signal, settings)
%   Decodes the demodulated signal according to settings and return the
%   derived information
% *****************************************************************************************

function [data, linkinfo] = sensatag_decoding(signal, settings)
version = 'beta 3.1';

% *****************************************************************************************
% version system
if nargin == 0
    data = version;
    return 
end

% call version system 
version_system();

% ******************************************************************************************
% input parameter checks

% number of input parameters
if nargin < 2
    criterr('No enough input arguments');
end

% length of vectors
if isempty(signal)
    err('Length of input signalis zero.');
end

% check contents of settings
%   prepare required data
expected.name = 'settings';
expected.reqfields = {'fclk'};
%   check
errortext = contentcheck(settings, expected);
%   output
if ~isempty(errortext)
    err('Incomplete settings\n%s', errortext);
end

% *******************************************************************************************
% scan input data vector and get timings

% get to first falling edge(start of delimiter)
startindex = 1;
while ~(signal(startindex) == 1 && signal(startindex+1) == 0)
    startindex = startindex + 1;
end

% detect the rising edge
rising_edges = [];
for i = startindex : length(signal)-1
    if signal(i) == 0 && signal(i+1) == 1
        rising_edges = [rising_edges; i];
    end
end

% delays between rising edges(in smaples)
delays = diff(rising_edges);

% safety check
if length(delays) < 6
    warn('Did not detect any supported command in demodulated mode.');
    data = [];
    linkinfo = struct('tari_s',NaN, 'tari', NaN, 'rtcal_s', NaN, 'rtcal',NaN, 'trcal_s', NaN,...
      'trcal', NaN, 'threshold_s', NaN, 'cmd','');
   return
end

% obtain necessary data (in period clk frequency)
linkinfo.tari_s = delays(1);
linkinfo.rtcal_s = delays(2);
if delays(3) > delays(2)
    linkinfo.trcal_s = delays(3);
    datastart = 4;
else
    linkinfo.trcal_s = NaN;
    datastart = 3;
end

% the same stuff in s 
linkinfo.tari  = linkinfo.tari_s / settings.fclk;
linkinfo.rtcal = linkinfo.rtcal_s / settings.fclk;
linkinfo.trcal = linkinfo.trcal_s / settings.fclk;

% ****************************************************************************************
% simple threshold detection
linkinfo.threshold_s = linkinfo.rtcal_s / 2;
data = double(delays(datastart:end) > linkinfo.threshold_s);
data = data(:);

% ****************************************************************************************
% decode command

% query command
if all(data(1:4) == [1;0;0;0]) && (length(data) >= 22)
    linkinfo.cmd      = 'query'; % command
    linkinfo.dr       = bit2opt(data(5),{8, 64/3}); % TRcal divide ration DR
    linkinfo.m        = bit2opt(data(6:7),{1,2,4,8}); % cycles per symbol M
    linkinfo.trext    = data(8); % pilot tone
    linkinfo.sel      = bit2opt(data(9:10),{'Al1', 'Al1', '~SL', 'SL'}); % which tags should respond to query
    linkinfo.session  = bit2opt(data(11:12),{'S0','S1','S2','S3'}); % session for inventory round
    linkinfo.target   = bit2opt(data(13),{'A','B'}); % inventoired flag A/B
    linkinfo.q        = bit2opt(data(14:17)); % q value
    linkinfo.crc5     = data(18:22); % crc-5
    linkinfo.crc5_ok   = (sum(crc(data,5)) == 0); % CRC-5 check
    
% ack command
elseif all(data(1:2) == [0 ; 1]) && (length(data) >= 18)
    linkinfo.cmd = 'ack';
    linkinfo.rn16 = dec2hex(bit2opt(data(3:18)), 4);
else
    linkinfo.cmd = '';
    warn('Truncated signal, unsupported command.');
end
end

