% *******************************************************************************************************
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
% Verison = sensatag_clock()
%   Just returns the version number
%
% nt = sensatag_clock (settings)
%   Return a vector containing sampling indices
%*********************************************************************************************


function nt = sensatag_clock(settings)
version = 'beta 3.1';

% ********************************************************************************************
% version system

% just return version number
if nargin == 0
    nt = version;
    return
end

% call version system
version_system();

% ********************************************************************************************
% input parameter checks

% check contents of settings
% prepare required data
expected.name = 'settings';
expected.reqfields = {'fcenter', 'fsigma', 'phi0', 'fs', 'length', 'mode'};

% check 
errortext = contentcheck( settings, expected);
% output
if ~isempty(errortext)
    err('Incomplete settings\n%s', errortext);
end

% settings.length should be natural number
if round(settings.length) ~= settings.length || settings.length < 0
    criterr('settings.length should be a natural number > 0');
end 

% settings.length == 0
if settings.length == 0
    warn('A clock vector of length zero was requested. Returing an empty vector.');
    nt = [];
    return
end

% ***********************************************************************************************
% create sampling intervals (samples @fs)

% normalized times
tcenter = settings.fs / settings.fcenter;
tsigma  = tcenter * settings.fsigma / 100;

% phase shift -> delay
if settings.phi0 < 0 % if random
   settings.phi0 = 360 * rand;
end
settings.phi0 = mod(settings.phi0, 360); % truncate to one period
t0 = settings.phi0 / 360 * tcenter;

% create array
switch lower(settings.mode)
   case 'fs'
      % first and last entry: no jitter to avoid nt < 0 and nt > settings.nmax (at least for same tsigma)
      length = floor(settings.length/tcenter);
      nt = 1 + round( t0 + (0:1:length-1)'*tcenter + [0; randn(length-2,1); 0] * tsigma );
   case 'fclk'
      % first entry: no jitter to avoid nt < 0 (at least for same tsigma)
      length = settings.length;
      nt = 1 + round( t0 + (0:1:length-1)'*tcenter + [0; randn(length-1,1)] * tsigma );
   otherwise
      err('Unknown settings.mode = "%s"', settings.mode);
end

% check first and last entry (last entry only for mode 'fs': max(nt) <= settings.length)
if min(nt) < 1 || ( strcmp(settings.mode, 'fs') && max(nt) > settings.length )
   err('Generated sampling intervals (indices) contain entries( settings.nmax < nt(i) < 1). Check settings.');
end

% monotony check
if min(diff(nt)) <= 0
   critwarn('Generated sampling intervals (indices) are not monotonous. Check settings.');
end

