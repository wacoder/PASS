% *******************************************************************************************************
% Position Aware RFID Systems: The PARIS Simulation Framework
% ***********************************************************
% selftest: channel - directivity (antenna gains)
%
%
%
% ***** Copyright / License / Authors *****
% Copyright 2007, 2008, 2009, 2010, 2011 Daniel Arnitz
%   Signal Processing and Speech Communication Laboratory, Graz University of Technology, Austria
%   NXP Semiconductors Austria GmbH Styria, Gratkorn, Austria
% Copyright 2012 Daniel Arnitz
%   Reynolds Lab, Department of Electrical and Computer Engineering, Duke University, USA
%
% This file is part of the PARIS Simulation Framework.
%
% The PARIS Simulation Framework is free software: you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software Foundation, either
% version 3 of the License, or (at your option) any later version.
%
% The PARIS Simulation Framework is distributed in the hope that it will be useful, but WITHOUT ANY
% WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License along with the PARIS Simulation 
% Framework. If not, see <http://www.gnu.org/licenses/>.
%
%
% ***** Behavior *****
% version = test_channel_directivity()
%    Just returns the version number (string).
% sumoferrors = test_channel_directivity(sumoferrors)
%    Tests the function CHANNEL_DIRECTIVITY, displays the results in the command window and returns the 
%    overall amount of errors found.
%
%
% ***** Interface definition *****
% function sumoferrors = test_channel_directivity(sumoferrors);
%    sumoferrors    sum of errors found not including this fcn call
%
%    sumoferrors    sum of errors found including this fcn call (+1 for each erroneous tested
%                   functionality)
%
%
% ***** Changelog *****
% REVISION   DATE         USER        DESCRIPTION (! bugfixes, + addons, - removals, ~ otherwise)
% beta 1.0   2010-03-30   arnitz      ~ initial release
% beta 2.0   2010-09-01   arnitz      ~ testing release (unstable)
% beta 3.0   2012-05-07   arnitz      ~ partial bugfix release
%
%
% ***** Todo *****
% = add "full 3D" directivity patterns
%
% *******************************************************************************************************


function sumoferrors = test_channel_directivity(sumoferrors)
version = 'beta 3.0';


% *******************************************************************************************************
% version system

% just return version number
if nargin == 0
   sumoferrors = version;
   return
end

% call version system (logs version numbers)
version_system();


% *******************************************************************************************************
% tolerances

% tolerances
%  small gains are problematic for the relative error (limited resolution of gain patterns), large gains
%  for the absolute error (linear interpolation w. limited resolution) => treated separately
internalsettings.gain_bound = 1e-2; % relerr if gain > gain_bound, absolute error otherwise
internalsettings.tol_gain1  = 1e-2; % relative error
internalsettings.tol_gain2  = 1e-3; % absolute error


% *******************************************************************************************************
% initialize

% output
disp('   = channel_directivity **');
disp(sprintf('     checking gain +/-%g%% if expected gain > %g, |result-expected|<=%g otherwise',...
   internalsettings.tol_gain1*100, internalsettings.gain_bound, internalsettings.tol_gain2));

% load settings and expected results
data = loadmat('results_channel_directivity.mat', '     ');
settings     = data.settings;
partitioning = data.partitioning;
results      = data.results;


% *******************************************************************************************************
% run tests and evaluate

for i = 1 : length(results)
   
   % partitioning and output
   %     find index for partition struct (partitioning.indices is "end-of-block")
   for j = 1 : length(partitioning.indices)
      if i <= partitioning.indices(j)
         index = j;
         break;
      end
   end
   
   % display once for each new test block
   if i == 1 || ( index > 1 && i == partitioning.indices(index-1) + 1 )
      disp(sprintf('      - %i setup(s) "%s"', partitioning.runs(index), partitioning.names{index} ));
   end
     
   % call channel_directivity  
   gain = channel_directivity(settings{i});
           
   % compare to expected results
   %     reset error/errortext once for each new test block
   if i == 1 || ( index > 1 && i == partitioning.indices(index-1) + 1 )
      errors    = 0;
      errortext = '';
   end
   %     check
   if any(results{i}.gain > internalsettings.gain_bound)
      [errors, errortext] = check_result(i, gain( results{i}.gain > internalsettings.gain_bound ),...
         results{i}.gain( results{i}.gain > internalsettings.gain_bound ), errors, errortext,...
         'gain (>bound)', 'relerr', internalsettings.tol_gain1);
   end
   if any(results{i}.gain <= internalsettings.gain_bound)
      [errors, errortext] = check_result(i, gain( results{i}.gain <= internalsettings.gain_bound ),...
         results{i}.gain( results{i}.gain <= internalsettings.gain_bound ), errors, errortext,...
         'gain (<bound)', 'range', internalsettings.tol_gain2*[-1,1]);
   end
      
   % output (if current loop is last test in block)
   if i == partitioning.indices(index)
      if errors == 0
         disp('         ... passed');
      else
         disp(sprintf('         ... ERRORS: %s', errortext));
         sumoferrors = sumoferrors + 1;  
      end
   end   
end

