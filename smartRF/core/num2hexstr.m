% ****************************************************************************
% Position Aware RFID Systems: The PARIS Simulation Framework
% ****************************************************************************
% number vector to hex string
% 
% ****************************
% Copyrigth: Jing Wang, Identigo Lab, uOttawa
%
%*****************************************************************************


function hexstr = num2hexstr(inputvector)

% ***************************************************************************
% input parameter checks

% number of input parameters
if nargin < 1
    error('Not enough input arguments');
    return
end

% empty inputvector
if isempty(inputvector)
    hexstr = '';
    return;
end

for i = 1 : length(inputvector)
    switch inputvector(i)
        case 0
            hexstr(i) = '0';
        case 1
            hexstr(i) = '1';
        case 2
            hexstr(i) = '2';
        case 3
            hexstr(i) = '3';
        case 4
            hexstr(i) = '4';
        case 5
            hexstr(i) = '5';
        case 6 
            hexstr(i) = '6';
        case 7
            hexstr(i) = '7';
        case 8
            hexstr(i) = '8';
        case 9
            hexstr(i) = '9';
        case 10
            hexstr(i) = 'a';
        case 11
            hexstr(i) = 'b';
        case 12
            hexstr(i) = 'c';
        case 13 
            hexstr(i) = 'd';
        case 14
            hexstr(i) = 'e';
        case 15
            hexstr(i) = 'f';
        otherwise
            err('Not a HEX number%d\n',inputvector(i));
    end
end
end
        
    



