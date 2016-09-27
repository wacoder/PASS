% *******************************************************************************************************
% Proximity-detection-based Augmented RFID System (PASS)
%   based on Position Aware RFID Systems: The PARIS Simulation Framework
% ***********************************************************
% reader - main
%
% *******************************************************************************************************
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
% ***** Behavior *****
% version = reader_main()
%    Just returns the version number (string).
% settings = reader_main('initialize', settings)
%   complete and sanitize settings
% settings = reader_main('prep_query', settings)
%   prepare modulation of a query command  
% settings = reader_main('set_maxclen', settings)
%   set carrier length to maximum length
% output = reader_main('tx_carrier', settings)
%   create and send an unmodulated carrier
% output = reader_main('tx_data', settings)
%   modulate and send prepared command
% output = reader_main('rx', input, settings)
%   receive and demodulate signal

%
%
% ***** Interface definition *****
% function output = reader_main(cmd, input, settings)
%    cmd        string with command (mode of operation) ... see Behavior
%    input      input signal (for some commands)
%    settings   settings structs (one for each reader)
%
%    output     depending on command: settings struct, results struct, output signal
%
%
% ***** Changelog *****
% REVISION   DATE         USER        DESCRIPTION (! bugfixes, + addons, - removals, ~ otherwise)
% beta 3.1   2014-04-10   jing        initial release
% *******************************************************************************************************


function output = reader_main(varargin)
version = 'beta 3.1';


% *******************************************************************************************************
% version system

% just return version number
if nargin == 0
   output = version;
   return
end

% call version system (logs version numbers)
version_system();


% *******************************************************************************************************
% internal settings

% maximum attenuation of components of interest (max/min) by reader input stage before a warning is issued
internalsettings.rxatt_max = 16; % dB (6 dB: power splitter!)


% *******************************************************************************************************
% input parameters

% number of input arguments
switch nargin
   case 2 % send
      cmd      = varargin{1};
      settings = varargin{2};
   case 3 % receive
      cmd      = varargin{1};
      input    = varargin{2};
      settings = varargin{3};
   otherwise
      err('Wrong amount of input parameters.');
end


% *******************************************************************************************************
% switch commands
switch lower(cmd)
   
   % ****************************************************************************************************
   % Transmitting process
    case 'main'
        switch nargin
            case 2 
                % Get the state of reader based on the previous state
                settings.reader.state = reader_control(settings.reader.state);
                % Get the transimtting signal based on current state
                settings.reader = reader_main(settings.reader.state, settings.reader);
            case 3
                % demodulate and decode the input signal 
                reader_rx = reader_receiver(input, settings.reader.receiver);
                reader_rx = reader_demodulation(reader_rx,settings.reader.demodulation, settings.reader.oscillator);
                    % settting for decode
                settings.reader.decode.dr = settings.reader.modulation.dr;
                settings.reader.decode.trcal = settings.reader.modulation.trcal;
                settings.reader.decode.trcal_s = settings.reader.modulation.trcal_s;
                settings.reader.decode.m = settings.reader.command.m;
                settings.reader.decode.fs = settings.reader.demodulation.fs;
                settings.reader.decode.frs = settings.reader.demodulation.frs;
                
                reader_decode = reader_decoded(reader_rx,settings.reader.decode);
                
                % Get the the state of reader based on the input and
                % current state
                settings.reader = reader_control(reader_decode, settings.reader);
                settings.reader = reader_main(settings.reader.state, settings.reader);
        end
        
        
        % find the maximum needed carrier length
        % **** Get the transmitting signal ****
                    %     data only
                    max_clen_s = ones(settings.tagpool.n+settings.sensatagpool.n,1) *...
                        ceil(max(cellfun(@(x) x.modulation.length, settings.reader)) * settings.fs); % [samples]
                    %     add the maximum overhead required by the tag implementation (group delays, ...)
                    for i = 1 : settings.tagpool.n
                        max_clen_s(i) = tag_main('clen_addoverhead', max_clen_s(i), settings.tag{i});
                    end
   
                    for i = 1 : settings.sensatagpool.n
                        max_clen_s(i+settings.tagpool.n) = sensatag_main('clen_addoverhead',...
                            max_clen_s(i+settings.tagpool.n), settings.sensatag{i});
                    end
                    %     and distribute to all readers (add field max_clen to settings)
                    settings.reader = cellfun(@(x) setfield(x, 'max_clen', max(max_clen_s)/settings.fs),...
                        settings.reader, 'UniformOutput',false); %#ok<SFLD>
               % modulate command
               headline('\nModulate');
               for i = 1 : settings.readerpool.n_nonvirt
                   if i == 1
                       headlin('    Reader %i', i);
                       % set carrier length to maximum needed length
                       settings.reader{i} = reader_main('set_maxlen', settings.reader{i});
                       % create modulated carrier
                       output{i} = reader_main('tx_data',settings.reader{1});
                   else
                       % create dummy
                       output{i} = [0];
                   end
               end      
 
   % MISC: complete and sanitize settings
   case 'initialize'
      % modulation
      settings.modulation = reader_modulation([0,0], settings.modulation); % even length for PR-ASK
      %     distribute modified settings
      settings.command.dr = settings.modulation.dr;
      % return modified settings
      output = settings;
   
   % ****************************************************************************************************
   % ACTIVE: prepare modulation of a query command
   case 'prep_query'
      %  encode command
      settings.data = reader_command('query', settings.command);
      %  Get the slot initialize value
      settings.slot_counter = 2^sum(settings.data(14:17).*[8 4 2 1]')-1;
      
      % get needed length of carrier to encode data and sanitize settings
      settings.modulation = reader_modulation(settings.data, settings.modulation);
      settings.oscillator.length = settings.modulation.length;
      % return settings
      output = settings;
      
   % ****************************************************************************************************
   % modulation of select command
    case 'selection'
        % initialize the command
        settings.command = NaN;
        
        % prepare the selection command
        
        settings.command.target = 's0';
        settings.command.action = [0;0;1];
        settings.command.length = 96;
        settings.command.mask   = settings.mask;
        settings.command.membank = 'epc';      
        
        
        % encode command
        settings.data = reader_command('selection',settings.command);
        settings.modulation = reader_modulation(settings.data, settings.modulation);
        settings.oscillator.length = settings.modulation.length;
        
        % return settings
        output = settings;
        
   % ***********************************************************************
   % prepare the acknwoledge command
    case 'pre_ack'
        % initialize the command
        settings.command = NaN;
        
        % prepare the ack command
        settings.command.rn16 = settings.rn16;
        
        % encode the command
        settings.data = reader_command('ack',settings.command);
        settings.modulation = reader_modulation(settings.data,settings.modulation);
        settings.oscillator.length = settings.modulation.length;
        
        % return settings
        output = settings;

   % ****************************************************************************************************
   % ACTIVE (MISC): set carrier length to maximum length
   case 'set_maxclen'
      settings.modulation.length = settings.max_clen;
      settings.oscillator.length = settings.max_clen;
      output = settings;
   
  
      
   % ****************************************************************************************************
   % ACTIVE: create and send an unmodulated carrier
   case 'tx_carrier'
      % create carrier for modulation
      output = reader_oscillator(settings.oscillator);
      % transmitter
      output = reader_transmitter(output, settings.transmitter);

   % ****************************************************************************************************
   % ACTIVE: modulate and send prepared command
   case 'tx_data'
      % create carrier for modulation
      output = reader_oscillator(settings.oscillator);
      % modulate
      output = reader_modulation(output, settings.data, settings.modulation);
      % transmitter
      output = reader_transmitter(output, settings.transmitter);
      
 
   % ****************************************************************************************************
   % PASSIVE: receive and demodulate signal
   case 'rx'
      % reader input stage (bandpass and power splitter)
      reader_rx = reader_receiver(input, settings.receiver);
      % IQ demodulation and sampling
      output = reader_demodulation(reader_rx, settings.demodulation, settings.oscillator); 
      
   % ****************************************************************************************************
   % transfer tag or sensatag backscatter information to reader decode component
    case 'set_decoded'
        settings.decode.dr = settings.modulation.dr;
        settings.decode.trcal = settings.modulation.trcal;
        settings.decode.trcal_s = settings.modulation.trcal_s;
        settings.decode.m = settings.command.m;
        settings.decode.fs = settings.demodulation.fs;
        settings.decode.frs = settings.demodulation.frs;
        
        output = settings;
   
   % *****************************************************************************************************
   % Decode from the demodulation signal of backscattering  
      
    case 'decoded'
        output = reader_decoded(input, settings.decode);

  

   % ****************************************************************************************************  
   otherwise
      err('Unsupported command: "%s".', lower(cmd));
end


return
    
% *******************************************************************************************************
% *******************************************************************************************************
% DEBUGGING SNIPPETS

% settings.ranging.mfcw_compsel.iirord = 2;
% settings.ranging.mfcw_compsel.att = 80;
% [c_mi, c_i, c_im] = mfcw_compsel(input, settings.ranging.mfcw_compsel);
% 
% close all
% 
% figure; hold on;
% plot(real(input), 'b')
% plot(imag(input), 'r')
% hold off; grid on; title('input');
% 
% figure; set(gca, 'yscale', 'log'); hold on;
% plot(abs(c_mi));
% plot(ind, abs(c_mi(ind,:)), 'r');
% hold off; grid on; title('c_{mi}'); %ylim([1e-7, 2e-5]);
% 
% figure; set(gca, 'yscale', 'log'); hold on;
% plot(abs(c_im));
% plot(ind, abs(c_im(ind,:)), 'r');
% hold off; grid on; title('c_{im}'); %ylim([1e-7, 2e-5]);


% *******************************************************************************************************
% OLD AND RUSTY
