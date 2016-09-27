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
% version = sensatag_encoding()
%   just return the version number
% outputsignal = sensatag_encoding(inputsignal, settings)
%    Performs the sensatag encoding as defined in EPCglobal Class1 Gen2 v1.0.9, no CRC-16 is added 
%    (like outputsignal = sensatag_encoding(inputsignal, settings, false))
% outputsignal = sensatag_encoding(inputsignal, settings, add_crc16)
%    Performs the sensatag encoding as defined in EPCglobal Class1 Gen2 v1.0.9 and adds the CRC-16 if
%    add_crc16=true
% *****************************************************************************************


function outputsignal = sensatag_encoding(inputsignal, settings, add_crc16)
version = 'beta 3.1';

% ******************************************************************************************
% version system
if nargin == 0
    outputsignal = version;
    return
end

% call version system 
version_system();

% *******************************************************************************************
% internal settings

% symbol set
internalsettings.symbolset = [0; 1];

% basis function
internalsettings.basisfunctions = [1,1;1,-1;-1,1;-1,-1]';

% fm0 preamble
internalsettings.fm0.preamble_trext0 = [1;1; -1;1; -1;-1; 1;-1; -1;-1; 1;1];
internalsettings.fm0.preamble_trext1 = [repmat([1;-1],12,1) ; internalsettings.fm0.preamble_trext0];

% Miller preamble
internalsettings.miller.preamble = [0; 1; 0; 1; 1; 1];

   % for FM0 encoding
internalsettings.fm0.transitions    = [3,4; 2,1; 3,4; 2,1]';
   % for Miller encoding
internalsettings.miller.transitions = [4,2; 4,3; 1,2; 1,3]';

% *******************************************************************************************
% input parameters check

% number of input parameters
if nargin == 2
    add_crc16 = false;
elseif nargin < 2
    criterr('No enough input arguments.');
end

% length of vectors
if isempty(inputsignal)
    warn('Length of input signal is zero.');
end

% check contents of settings
%   prepare required data
expected.name = 'settings';
expected.reqfields = {'trext', 'm'};

%   check 
errortext = contentcheck(settings, expected);
%   output
if ~isempty(errortext)
    err('Incomplete settings\n%s',errortext);
end

% make all input vectors column vectors
inputsignal = inputsignal(:);

% *******************************************************************************************
% encoding(FM0 or Miller) plus preamble, End of signal and CRC16 if
% requested

% add CRC16 if requested
if add_crc16
    inputsignal = [inputsignal; crc(inputsignal, 16)];
end

% encoding
switch lower(settings.m)
    % FM0
    case 1
        % encoding of data plus adding of 'dummy 1' for End of signal
        outputsignal = fm0_statemachine([inputsignal;1], internalsettings);
        % add preamble
        if settings.trext == 0
            outputsignal = [internalsettings.fm0.preamble_trext0; outputsignal];
      else
         outputsignal = [internalsettings.fm0.preamble_trext1; outputsignal];
        end
    % Miller
    case {2,4,8}
       % encoding of data plus adding second part of preamble and "dummy 1" for End-of-Signaling
       outputsignal = miller_subcarrier( miller_statemachine([internalsettings.miller.preamble; inputsignal; 1], internalsettings), settings.m);
       % add first part of preamble
       if settings.trext == 0
          outputsignal = [repmat([1;0]*2-1, 4*settings.m, 1); outputsignal];
       else
          outputsignal = [repmat([1;0]*2-1,16*settings.m, 1); outputsignal];
       end

    otherwise
       err('Unsupported M: %s - only 1,2,4,8 (1=FM0; 2,4,8=Miller) supported.', settings.m);
end
end




% *******************************************************************************************************
% *******************************************************************************************************
% Implementation of FM0 generator state machine
function out = fm0_statemachine(in, internalsettings)

% initial state has to be s3 or s4 (falling edge from preamble)
initialstate = 3 + in(1);

% finite state machine
out = fsm(in, internalsettings.fm0.transitions, internalsettings.basisfunctions, initialstate, internalsettings.symbolset);

end



% *******************************************************************************************************
% *******************************************************************************************************
% Implementation of Miller-signaling state machine
function out = miller_statemachine(in, internalsettings)

% initial state has to be s1 or s2 (rising edge from first part of preamble)
initialstate = 1 + in(1);

% finite state machine
out = fsm(in, internalsettings.miller.transitions, internalsettings.basisfunctions, initialstate, internalsettings.symbolset);


end


% *******************************************************************************************************
% *******************************************************************************************************
% Multiplies basis functions in Miller coding with square wave
function out = miller_subcarrier(in, M)

% define signals
subcarrier = repmat([1;-1], M/2, 1);
out = zeros(M*length(in), 1);

% multiplication
% is there a better (faster) implementation [???]
for i = 1 : length(in)
   out((i-1)*M+1:i*M) = in(i) * subcarrier;
end

end