%***********************************************************************************************************************************************
% Proximity-detection-based Augmented RFID System (PASS)
%   based on Position Aware RFID Systems: The PARIS Simulation Framework
% ***********************************************************
%
% reader decode
%
% ***** Copyright / License / Authors *****
% Copyright Jing Wang Identigo Lab
%
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
%************************************************************************************************************************************************
%
%
% ********* Behavior *********
% function sig_d = reader_decoded( signal, settings)
% sig_d           decoded signal
% signal          complex baseband signal 
% settings        struct containging decoding settings
%
%************************************************************************************************************************************************
% ***** Changelog *****
% REVISION   DATE         USER        DESCRIPTION (! bugfixes, + addons, - removals, ~ otherwise)
% beta 3.1   2014-04-10   jing        initial release


function sig_d = reader_decoded(varargin)
version = 'beta 3.1';

if nargin == 0
    sig_d = version;
    return
end 

% call version system
version_system();

if nargin == 2
    signal_bb = varargin{1}(:);
    settings   = varargin{2};
else 
    criterr('Wrong number of input arguments.');
end 

% check contents of settings
% %     prepare required data
% expected.name = 'settings';
% expected.reqfields = {'iirord', 'att', 'fcut', 'frs', 'fs', 'q', 'len_rs'};
% %     check
% errortext = contentcheck(settings, expected);
% %     output
% if ~isempty(errortext)
%    err('Incomplete settings\n%s', errortext);
% end

% absolute value of complex signal

sig_bbs = abs(signal_bb);
sig_bbs = sig_bbs-mean(sig_bbs);
signal = [];

switch settings.m
    
    case 1
        tpri = settings.trcal/settings.dr;
        tpri_s = floor(settings.trcal_s/settings.dr);
        % oversampling in demodulation
        tpri_s = floor(tpri_s/(settings.fs/settings.frs));
        
        % substract the unstationary phase
        sub_phase = floor(tpri_s/2);
        sig_bbs(1:sub_phase) = 0;
        sig_bbs = sig_bbs - mean(sig_bbs);

        % polar coding
        sig_polar = double(sig_bbs>0);
        
        % downsampling rate
        linktime = tpri_s;
        % signal flow process
        cnt = 0;
        cnt_en = 0;
        ac_rc =[1 0 1 0];
        enable = [];
        for i = 1: length(sig_polar)-1
            if xor(sig_polar(i),sig_polar(i+1))
                cnt_en = 1;
            end
            
            if cnt_en == 1
                if cnt == linktime
                    cnt = 1;
                else
                    cnt = cnt + 1;
                    if cnt == floor(linktime/4) || cnt == floor(linktime*3/4)
                        signal = [signal, sig_polar(i)];
                        ac_rc = [ac_rc(2:4),sig_polar(i)];
                        
                    end
                end
            else
                cnt = 0;
                
            end
            
            if max(ac_rc) == min(ac_rc)
                cnt_en = 0;
            end
            
        end
        
        data_en = 0;
        sig_output = [];
        sig_cnt = 0;
        head = [];
        violation = 0;
        
        % the head of FM0
        for i = 1:length(signal)-9
            head = signal(i:i+9);
            if head(1)==head(2) && head(3)~=head(4) &&...
                    head(5)==head(6) && head(7)~=head(8) &&...
                    head(2)~=head(3) && head(4)~=head(5) &&...
                    head(6)~=head(7) && max(head(8:end))==min(head(8:end))
                violation = i+10;
                break;
            end
        end
        signal = signal(violation:end);
        
        for i = 1:2:length(signal)-2
            if signal(i) == signal(i+1)
                sig_output = [sig_output,1];
            else
                sig_output = [sig_output,0];
            end   
              
            % dummy '1' if dummy 1 ends at high level this would be also
            % effective 
            if signal(i+1) == signal(i+2)
                warn('This is the end of data or inactivity');
                break;
            end
        end
        sig_d = sig_output;
    case {2,4,8}
        tpri = settings.trcal/settings.dr;
        tpri_s = floor(settings.trcal_s/settings.dr);
        % oversampling in demodulation_ according to reader 
        tpri_s = floor(tpri_s/(settings.fs/settings.frs));
        
        % substract the unstatinary phase
        sig_bbs(1:tpri_s) = 0;
        sig_bbs = sig_bbs - mean(sig_bbs);
        
        % polar coding
        sig_polar = double(sig_bbs>0);
        
        % initiaiization
        linktime = tpri_s;
        pr = '001';
        
        
        for i= 1:length(sig_polar)-1
            switch pr
                % initialization
                case '001'
                    if xor(sig_polar(i),sig_polar(i+1))
                        pr = '010';
                        cnt = 1;
                        num_narrow_blocks = 0;
                    end
                
                % for robustness, after getting 5 narrow blocks
                case '010'
                    if xor(sig_polar(i),sig_polar(i+1))
                        if cnt > linktime/4 && cnt < linktime*3/4
                            num_narrow_blocks = num_narrow_blocks + 1;
                            cnt = 1;
                            if num_narrow_blocks == 5
                                pr = '100'
                            else
                                pr = '010';
                            end
                        else
                            cnt = 1;
                            pr = '010';
                        end
                    else
                        cnt = cnt + 1;
                    end
                
                % adjust the linktime samples per tpri, and detect the
                % beginning of data
                case '100'
                    if xor(sig_polar(i),sig_polar(i+1))
                        if cnt > linktime/4 && cnt < linktime*3/4
                            linktime = floor(linktime/2 + cnt);
                            pr = '100';
                            cnt = 1;
                        elseif cnt > linktime*3/4 && cnt < linktime*5/4
                            pr = '011';
                            count = 0;
                            cnt = 1;
                        else
                            pr = '100';
                            cnt = 1;
                        end
                    else
                        cnt = cnt + 1;
                    end
                
                % jump to the head '0111'
                case '011' 
                    if xor(sig_polar(i),sig_polar(i+1))
                        count = count + 1;
                        cnt = 1;
                        
                        if count == settings.m-1
                            pr = '110';
                            data_sp = [];
                        end                         
                    else
                        cnt = cnt + 1;
                    end
                % sampling the signal 2 per bit
                case '110'
                    if xor(sig_polar(i),sig_polar(i+1))
                        if cnt > linktime/4 && cnt < linktime*3/4
                            data_sp = [data_sp sig_polar(i-floor(linktime/4))];
                        elseif cnt > linktime*3/4 && cnt < linktime*5/4
                            data_sp = [data_sp sig_polar(i-floor(linktime*3/4)) sig_polar(i-floor(linktime/4))];
                        elseif cnt > linktime*5/4
                            pr = '001';
                        end
                        cnt = 1;
                        pr = '110';
                    else
                        cnt = cnt + 1;
                    end
                otherwise
                    err('unanticipated situation');
                    
            end
        end
        
        % generate the local miller subcarrier
        switch settings.m
            case 2
                miller_zero = [1 0 1 0];
                miller_one = [1 0 0 1];
            case 4
                miller_zero = [1 0 1 0 1 0 1 0];
                miller_one  = [1 0 1 0 0 1 0 1];
            case 8
                miller_zero = [1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0];
                miller_one  = [1 0 1 0 1 0 1 0 0 1 0 1 0 1 0 1];
            otherwise
                err('unsupported Miller m');
        end
        
        data_out = [];
        
        % correlation between signal and local subcarrier, decode the data
        for i = 1:2*settings.m:length(data_sp)
            if length(data_sp) < i+2*settings.m-1
                signal_sp = data_sp(i:end);
                miller_zero = miller_zero(1:length(data_sp)-i+1);
                miller_one  = miller_one(1:length(data_sp)-i+1);
            else
                signal_sp = data_sp(i:i+2*settings.m-1);
            end
            
            cor_zero = cor(signal_sp,miller_zero);
            cor_one  = cor(signal_sp,miller_one);
            
            if cor_one > cor_zero
                data_out = [data_out 1];
            else 
                data_out = [data_out 0];
            end
        end
        
        % check the head
        if data_out(1:4)== [0 1 1 1]
            sig_d = data_out;
        else
            sig_d = [];
        end
            
            
            
        
        

                    
                            
%                 % check 
%                 if (cnt > floor(linktime/4)||cnt < floor(linktime*3/4))
%                     num_narrow_blocks = num_narrow_blocks + 1;
%                 end
%                 % update the linktime
%                 if num_narrow_blocks == 5
%                     linktime = floor(linktime/2 + cnt);
%                 end
%                 
%                 
%                 if (cnt)
%                 
%                 cnt = 1;
%                 
                    
                    
        
        
        
        
                
                
        
            
                
            
            

    
    otherwise
    err('Unsupported, Not FM0 or miller 2,4,8: %s', settings.m);

end
end

function cor_value = cor(va,vb)
va = 1 - 2*va;
vb = 1 - 2*vb;
cor_value = abs(sum(va.*vb));
end





    

