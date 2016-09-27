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
% version = sensatag_main()
%  just returns the version number
% settings =sensatag_main('initialize',settings)
%  complete the sensatag settings 
%
%
% *****************************************************************************************


function output = sensatag_main(varargin)
version = 'beta 3.1';

% *****************************************************************************************
% version system

% just return version number
if nargin == 0
    output = version;
    return
end

% call version system(log version number)
version_system();

% number of input arguments
switch nargin
    case 2 % send
        cmd       = varargin{1};
        settings  = varargin{2};
    case 3 % receive
        cmd       = varargin{1};
        input     = varargin{2};
        settings  = varargin{3};
    otherwise
        err('Wrong amount of input parameters');
end

% *****************************************************************************************
% switch commands
switch lower(cmd)
    
    % *************************************************************************************
    %  initialize: complete and sanitize settings
    case 'initialize'
        settings.modulation = sensatag_modulation([0], settings.modulation);
        % the sensatag doesn't need power for chips
        
        settings.state.epc       = ' ';
        if isfield(settings.state, 'linkinfo')
            settings.state = rmfield(settings.state, 'linkinfo');
        end
        % return modified settings
        output = settings;
    % *************************************************************************************
    % add modulation overhead to carrier length
    case 'clen_addoverhead'
        % add the overhead required by the sensatag implementation
        output = input + settings.modulation.grpdel_s + 2*settings.modulation.t0_s; % [samples]
        % make carrier length a multiple of largest filterbank size (required by tag_modulation)
        output = output + settings.modulation.nfft - mod(output, settings.modulation.nfft);
        
    % **************************************************************************************    
    case 'rx'
        % receive the transmitted signal, input to chip
        [output.rxsignal, sensatagmod_internal] = sensatag_modulation( input, [], [], settings.modulation);
        % modify the settings
        output.settings = settings;
    case 'rx_data'
        % create sensatag clock( sampling mode)
        settings.clock.mode = 'fs';
        settings.clock.length = length(input);
        clock = sensatag_clock(settings.clock);
        % demodulate and sample
        sensatag_demodulated = sensatag_demodulation(input, clock, settings.demodulation);
        [output.decoded, output.linkinfo] = sensatag_decoding(sensatag_demodulated.demod, settings.decoding);
    case 'prep_rn16'
        settings.id.rn16 = num2str(randi(16,1,4));
        % encode the command
        settings.encdata = (sensatag_encoding(hex2bit(settings.id.rn16), settings.encoding, false) + 1) / 2;
        % determine the length and santize settings
        settings.modulation = sensatag_modulation(settings.encdata, settings.modulation);
        output = settings;
    % for the phase cancellation observation
    case 'rx_power'
        % receive the transmitted signal
        [rxsignal, sensatagmod_internal] = sensatag_modulation( input, [], [], settings.modulation);
        indication = 1:floor(length(rxsignal)*3/3.5); 
        rxsignal = rxsignal(indication);
        % create sensatag clock 
        settings.clock.mode = 'fs';
        settings.clock.length = length(rxsignal);
        clock = sensatag_clock(settings.clock);
        % demodulate and sample
        % adjust the agc
        settings.demodulation.ar = settings.demodulation.af;
        settings.demodulation.af = -settings.demodulation.af;
        sensatag_demodulated = sensatag_demodulation(rxsignal, clock, settings.demodulation);
        output.signal = sensatag_demodulated.signal;
        sig_bbs = sensatag_demodulated.demod;
        % sampling the baseband signal
        sig_polar = sig_bbs(12:20:end);
        output.demod = sig_polar(1:end-1);
    
    case 'rx_mix'
        % receive the transmitted signal, input to chip
        [rxsignal, sensatagmod_internal] = sensatag_modulation( input, [], [], settings.modulation);
        
        % create sensatag clock 
        settings.clock.mode = 'fs';
        settings.clock.length = length(rxsignal);
        clock = sensatag_clock(settings.clock);
        % demodulate and sample
        sensatag_demodulated = sensatag_demodulation(rxsignal, clock, settings.demodulation);
        %output = rxsignal;
        %[output.decoded, output.linkinfo] = sensatag_decoding(sensatag_demodulated, settings.decoding);
        %
        settings.decode.dr = settings.state.linkinfo.dr;
        settings.decode.trcal = settings.state.linkinfo.trcal;
        settings.decode.m = settings.state.linkinfo.m;
        settings.decode.fs = settings.demodulation.fs;
        settings.decode.dsf = settings.clock.fs/settings.clock.fcenter;
        
        % sensatag decode
        output = sensatag_bcsk_decoding(sensatag_demodulated, settings.decode);
        
        
        
      
    otherwise
        err('Unsupported command:"%s"',lower(cmd));
end
        





end

