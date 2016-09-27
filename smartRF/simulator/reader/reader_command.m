% *******************************************************************************************************
% Proximity-detection-based Augmented RFID System (PASS)
%   based on Position Aware RFID Systems: The PARIS Simulation Framework
% ***********************************************************
% reader - create commmand data stream (query, ack, selection, query_adj, 
% queryrep, Nak)
%
% ***** Copyright / License / Authors *****
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
% version = reader_command()
%    Just returns the version number (string).
% data = reader_command(command, settings)
%    Returns the bitstream for COMMAND according to SETTINGS.
%    ATTENTION: SETTINGS not checked for conformity/validity/...!
%
%
% ***** Interface definition *****
% data = reader_command(command, settings)
%    command    string defining the command {'query', 'ack', 'selection', 
%               'query_adj', 'queryrep', 'Nak'}
%    settings   parameter for different command
%
%    data       binary bitstream for command according to settings
%
%
% ***** Changelog *****
% beta 3.1   2014-04-10   jing        initial release
% ***** Todo *****
%
% *******************************************************************************************************

function data = reader_command(command, settings)
version = 'beta 3.1';


% *******************************************************************************************************
% version system

% just return version number
if nargin == 0
   data = version;
   return
end

% call version system (logs version numbers)
version_system();


% *******************************************************************************************************
% input parameter checks / prepare input parameters

% number of input parameters
if nargin < 2
   criterr('Not enough input arguments.');
end


% *******************************************************************************************************
% commands

switch lower(command)
   
% query
   case 'query'
      % check contents of settings
      %     prepare required data
      expected.name = 'settings';
      expected.reqfields = {'dr', 'm', 'trext', 'sel', 'session', 'target', 'q'};
      %     check
      errortext = contentcheck(settings, expected);
      %     output
      if ~isempty(errortext)
         err('Incomplete settings\n%s', errortext);
      end
      % create command bitstream
      data = zeros(22,1);
      data(1:4) = [1; 0; 0; 0]; % command code
      switch settings.dr % TRcal divide ratio
         case 8
            data(5) = 0;
         case 64/3
            data(5) = 1;
         otherwise
            err('Unsupported DR "%f"', settings.dr);
      end
      switch settings.m % cycles per symbol
         case 1
            data(6:7) = [0; 0];
         case 2
            data(6:7) = [0; 1];
         case 4
            data(6:7) = [1; 0];
         case 8;
            data(6:7) = [1; 1];
         otherwise
            err('Unsupported M "%f"', settings.m);
      end
      data(8) = settings.trext; % pilot tone
      switch lower(settings.sel) % which tags should respond to query
         case 'all'
            data(9:10) = [0; 0];
         case '~sl'
            data(9:10) = [1; 0];
         case 'sl'
            data(9:10) = [1; 1];
         otherwise
            err('Unsupported sel "%s"', settings.sel);
      end
      switch lower(settings.session) % session for inventory round
         case 's0'
            data(11:12) = [0; 0];
         case 's1'
            data(11:12) = [0; 1];
         case 's2'
            data(11:12) = [1; 0];
         case 's3'
            data(11:12) = [1; 1];
         otherwise
            err('Unsupported target "%s"', settings.target);
      end
      switch lower(settings.target) % inventoried flag A/B
         case 'a'
            data(13) = 0;
         case 'b'
            data(13) = 1;
         otherwise
            err('Unsupported target "%s"', settings.target);
      end
      data(14:17) = bitget(settings.q, 4:-1:1); % q value
      data(18:22) = crc(data(1:17), 5); % CRC-5
      
% ACK
   case 'ack'
      % check contents of settings
      %     prepare required data
      expected.name = 'settings';
      expected.reqfields = {'rn16'};
      %     check
      errortext = contentcheck(settings, expected);
      %     output
      if ~isempty(errortext)
         err('Incomplete settings\n%s', errortext);
      end
      % create command bitstream
      data = zeros(18,1);
      data(1:2)  = [0; 1]; % command code
      data(3:18) = hex2bit(settings.rn16); % RN16

% SELECTION
    case 'selection'
        % check contents of settings
        expected.name = 'settings';
        expected.reqfields = {'target','action','membank','length','mask'};
        %   check
        errortext = contentcheck(settings, expected);
        %   output
        if ~isempty(errortext)
            err('Incomplete settings\n%s',errortext);
        end
        
        % create the command bitstream
        data = zeros(142,1);
        data(1:4)=[1;0;1;0]; % command code for selection
        
        % target
        switch lower(settings.target)
            case 's0'
                data(5:7) = [0;0;0];
            case 's1'
                data(5:7) = [0;0;1];
            case 's2'
                data(5:7) = [0;1;0];
            case 's3'
                data(5:7) = [0;1;1];
            case 'sl' % SL
                data(5:7) = [1;0;0];
            otherwise
                err('Unsupported target command\n%s', settings.target);
        end
        
        % action 
        data(8:10) = settings.action;
        
        % membank, pointer, length is not functional, Mask which is epc 
        % directly compare with tag's epc (tag's memory management to do)  
        % membank
        switch lower(settings.membank)
            case 'rfu'
                data(11:12) = [0;0];
            case 'epc'
                data(11:12) = [0;1];
            case 'tid'
                data(11:12) = [1;0];
            case 'user'
                data(11:12) = [1;1];
            otherwise
                err('Unsupported Membank command\n%s', settings.membank);
        end
        
        % pointer 
        data(13:20) = zeros(8,1);
        
        % length
        length = dec2binvec(settings.length,8);
        data(21:28) = length(:);
        
        % mask(epc)
        data(29:124) = hex2bit(settings.mask);
        
        % truncate
        % all is disable currectly
        %data(125) = settings.truncate;
        data(125) = 0;
        
        % crc16
        data(126:141) = crc(data(1:125),16);

% query_adj
    case 'query_adj'
        
        % check content
        expected.name = 'settings';
        expected.reqfields = {'session','updn'};
        
        % check
        errortext = contentcheck(settings, expected);
        
        % output
        if ~isempty(errortext)
            err('Incomplete settings\n%s',errortext);
        end
        
        % create command bitstream
        data = zeros(10,1);
        
        data(1:4) = [1;0;0;1]; % command code for query_adjust
        
        % session
        switch lower(settings.session)
            case 's0'
                data(5:6) = [0;0];
            case 's1'
                data(5:6) = [0;1];
            case 's2'
                data(5:6) = [1;0];
            case 's3'
                data(5:6) = [1;1];
            otherwise
                err('Unsupported Session command\n%s',settings.session);
        end
        
        % updn
        data(7:9) = settings.updn;

% QueryRep
    case 'queryrep'
        
        % chcek content
        expected.name = 'setings';
        expected.reqfields = {'session'};
        
        % check
        errortext = contentcheck(settings,expected);
        % output
        if ~isempty(errortext)
            err('Incomplete settings\n%s',errortext);
        end
        
        data = zeros(4,1);
        
        % create command bitstream
        data(1:2) = [0;0];
        
        % session
        switch lower(settings.session)
            case 's0'
                data(3:4) = [0;0];
            case 's1'
                data(3:4) = [0;1];
            case 's2'
                data(3:4) = [1;0];
            case 's3'
                data(3:4) = [1;1];
            otherwise
                err('Unsupported Session command\n%s', settings.session);
        end

% NAK
    case 'nak'
        data = [1;1;0;0;0;0;0;0]; % command code for NAK
        
        
        
        
        
        
        
        
        
   otherwise
      err('Unsupported command "%s"', command);
end


