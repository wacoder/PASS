% *******************************************************************************************************
% Proximity-detection-based Augmented RFID System (PASS)
%   based on Position Aware RFID Systems: The PARIS Simulation Framework
% ***********************************************************
% reader control, state machine on controlling the reader's state
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
%
%
% **** Behavior ****
% version = reader_control()
%  just returns the version number
% state = reader_control()
%   returns the reader's state according to input decoded signal
% 
% ***** Change log ****
% beta 3.1   2014-04-10   jing        initial release
% *******************************************************************************************************

function output = reader_control(varargin)
version = 'beta 3.1';

% just return the system number
if nargin == 0
    output = version;
    return 
end

% number of input paramters
switch nargin
    case 1 
        settings = varargin{1};
    case 2
        input    = varargin{1}
        settings = varargin{2};
    otherwise
        err('Wrong amount of input parameters');
end

switch lower(settings.state)
    % initialize the reader
    case ''
        settings.state = 'initialize';
    % send selection
    case 'initialize'
        settings.state = 'selection';
    % send query
    case 'selection'
        settings.state = 'query';
    case 'query'
        % no reply
        if nargin == 1
            if settings.slot_counter == 0
                % if sel_enable
                % settings.state = 'selection';
                % else
                % settings.state = 'queryadj'
                % end
            elseif settings.slot_counter > 0
                settings.slot_counter = settings.slot_counter - 1;
                settings.state = 'queryrep';
            else
                err('Unsupported reader slot counter %s', settings.slot_counter);
            end
        else
            % if collision
            %   if settings.slot_counter == 0
            %       if sel_enable
            %           settings.state = 'selection'
            %       else
            %           settings.state = 'queryadj'
            %       end
            %   elseif settings.slot_counter > 0
            %       settings.slot_counter = settings.slot_counter - 1;
            %       settings.state = 'queryrep';
            %   else
            %       err('Unsupported reader slot counter %s',settings.slot_coutner)
            % else (no collison)
            % if length(input)>20
                    settings.command.rn16 = input(5:20);
                    settings.state = 'ack';
        end
        
        
    case 'ack'
        % no reply
        if nargin == 1
            settings
            
            



end

