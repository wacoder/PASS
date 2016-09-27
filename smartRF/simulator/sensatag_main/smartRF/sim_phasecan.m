% ************************************************************************************************************************
% Proximity-detection-based Augmented RFID System Simulator(PASS)
% ***************************************************************
% sim_phasecan.m : the behavioral description script 
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
% *** Behavior ***
% This file proivdes the behavior description, 
% 1. configure the channel according to transferred settings from user
%    description script
% 2. establish the communication link according to EPCglobal class1 Gen2
%
% ***** change log *****
% beta 3.1    2014-04-10   jing    initial release
%
% *************************************************************************
% 
% ***** Todo **********
% 1. Q selection and sensatag protocol(optional)
% 2. without the selection algorithm, the whole process of query: query->
%    RN16->ACK->EPC->repQuery or query->RN16->ACK->->NAK or Query->collison->repQuery 
%
% **************************************************************************************************************************



function [settings] = sim_phasecan( settings )
version = 'beta 3.1';

% *******************************************************************************************************
% version system

% just return version number
if nargin == 0
   settings = version;
   return
end

% *******************************************************************************************************
% internal settings

% changes of fs to avoid low beat frequencies caused by sampling (for time-variant energy detection)
internalsettings.bnd_osr =   10; % if oversampling rate is smaller that bnd_ord ...
internalsettings.tol_osr = 1e-1; % ... it may not be closer to an integer than tol_osr
%internalsettings.tol_osr = 0.5; % ... test the fix of sampling frequency 

% *******************************************************************************************************
% logging

% start timer and diary
tic;
settings.diaryfilename = fullfile(settings.folders.root, settings.folders.logs, sprintf('%s.log', settings.suffix)); 
system(sprintf('erase %s', settings.diaryfilename));% (do not append diary)
diary(settings.diaryfilename);

% create logfile header 
disp('*******************************************************************************************************');
disp('*******************************************************************************************************');
disp('***');
disp(sprintf('*** Created by %s.m, version %s, %s on host "%s"',...
   mfilename, version, datestr(now, 'local'), settings.hostname));
disp('***');
disp('*******************************************************************************************************');
disp('*******************************************************************************************************');

% access globalsettings
global globalsettings;



% *******************************************************************************************************
% initialize simulator, load standard setup and create individuals
headline('\n\n');
headline('*******************************************************************************************************');
headline('* Initializing Simulator and Creating Individuals');

% make sure oversampling rate ist not close to an integer
% (would result in very low beat frequency which cannot be handled by
% tag_demodulation)，过采样频率的范围，并且不能是整数倍
osr = settings.fs./cell2mat(settings.readerpool.fc);
if any( and(osr < internalsettings.bnd_osr, mod(osr,1) < internalsettings.tol_osr) )
   critwarn('Low oversampling rate for carrier is close to an integer. Increasing sampling frequency.');
     % try to fix
   for i = 1 : ceil(1/internalsettings.tol_osr) - 1
      settings.fs = settings.fs + min(cell2mat(settings.readerpool.fc)) * internalsettings.tol_osr;
      osr = settings.fs./cell2mat(settings.readerpool.fc);
      if ~any( and(osr < internalsettings.bnd_osr, mod(osr,1) < internalsettings.tol_osr) )
         break;
      end
   end
end 
 % check one last time
   if any( and(osr < internalsettings.bnd_osr, mod(osr,1) < internalsettings.tol_osr) )
      err('Unable to fix oversampling rate problem.');
   end



% output
disp(sprintf('\nPlease note:'));
%     renew tickets only if not in probemode (loop exec. time in probemode is in the range of seconds or below)
if settings.specials.probemode && settings.renew_kticket
   settings.renew_kticket = false;
   disp(sprintf('   .) Renewal of Kerberos/AFS tickets has been deactivated (Field Probe Mode).'));
end
%     settings content check has been switched off
if ~globalsettings.core.settings_check
   disp(sprintf('   .) Settings content check has been switched off for performance reasons.'));
end
%     sampling resolution 
disp(sprintf('   .) Delays will be rounded to sampling interval, resolution is %g ns (%g cm).',...
   1e9/settings.fs, 100*settings.c/settings.fs));
%     logging "warnings"
if ~globalsettings.logging.warnings || ~globalsettings.logging.headlines || ~globalsettings.logging.messages
   disp(sprintf('\nThe following output is switched off:'));
    if ~globalsettings.logging.warnings
      disp('   .) Warning output. You will not receive any warnings!');
   end
   if ~globalsettings.logging.headlines
      disp('   .) Headlines and progress output (ETA, ...).');
   end
   if ~globalsettings.logging.messages
      disp('   .) Message output. You will not receive any detailed information.');
   end
end    

% load standard simulator setup
headline('\nLoading Standardsettings');
stdsettings = sim_stdsettings_mfcw(settings);

% load assembly/detuning state mapping (for all tags)
for i = 1 : settings.tagpool.n
   adsm{i} = load(settings.tagpool.adsm_file{i});
end
% load assembly/detuning state mapping (for all sensatags)
for i = 1 : settings.sensatagpool.n
    sadsm{i} = load(settings.sensatagpool.adsm_file{i});
end

stdsettings.channel.probemode      = settings.specials.probemode;
stdsettings.channel.smallscale.on  = settings.channel_global.small_on;
stdsettings.feedback.smallscale.on = settings.channel_global.small_on;
stdsettings.channel.smallscale.det  = settings.channel_global.small_det;
stdsettings.feedback.smallscale.det = settings.channel_global.small_det;
if ~settings.channel_global.noise_on %why here is a semicolon
   stdsettings.channel.noise.type  = 'off'; % should be off anyway
   stdsettings.feedback.noise.type = 'off';
else
   stdsettings.channel.noise.type  = 'wgn'; % this is different from original
   stdsettings.feedback.noise.type = 'wgn'; % cf. signal model
end

%     smallscale channel
stdsettings.channel.type   = settings.channel_global.type;
stdsettings.channel.v_dist = settings.channel_global.v_dist;
stdsettings.channel.v_k    = settings.channel_global.v_k;
stdsettings.channel.v_trms = settings.channel_global.v_trms;
%     noise
stdsettings.channel.noise.n0 = settings.channel_global.n0;

%     reflective surface(s)
if settings.channel_global.surf_on
   if ~isfield(settings.channel_global, 'surfaces')
      critwarn('Surfaces switched on in global channel settings, but none defined. Switching off.');
      settings.channel_global.surf_on = false;
   else
      stdsettings.channel.surfaces  = settings.channel_global.surfaces;
      stdsettings.feedback.surfaces = settings.channel_global.surfaces;
      %     copy polarization
      refl_names = fieldnames(stdsettings.channel.surfaces);
      for k = 1 : length(refl_names)
         stdsettings.channel.surfaces.(refl_names{k}).pol_dim  = settings.channel_global.pol_dim;
         stdsettings.feedback.surfaces.(refl_names{k}).pol_dim = settings.channel_global.pol_dim;
      end
      clear('refl_names');
   end
end

% readerpool
headline('\nReaderpool');
settings.readerpool.n_nonvirt = sum(cellfun(@(x) x==0, settings.readerpool.virt));
if find(cellfun(@(x) x==0, settings.readerpool.virt)==0, 1, 'first') - 1 ~= settings.readerpool.n_nonvirt
   err('Non-virtual transmitters have to be placed at the beginning of readerpool.');
end

if settings.readerpool.n_nonvirt < settings.readerpool.n && ~settings.specials.probemode
   critwarn('There is a bug in the handling of VTX in standard mode. Sorry.'); % partial fix implemented: reduced to warning
end

for i = 1 : settings.readerpool.n
   % copy from standardsettings
   settings.reader{i} = stdsettings.reader;
   
   % setup modulation and complete/sanitize the settings
   settings.reader{i}.modulation.t0 = settings.readerpool.t0{i};
   settings.reader{i} = reader_main('initialize', settings.reader{i});


   % reader component setup
   settings.reader{i}.oscillator.fcenter = settings.readerpool.fc{i};
   settings.reader{i}.analogpathest.f0   = settings.readerpool.fc{i};
   settings.reader{i}.transmitter.ptx    = settings.readerpool.ptx{i};
   settings.reader{i}.demodulation.frs   = settings.readerpool.frs{i};
   settings.reader{i}.demodulation.fcut  =4* settings.reader{i}.modulation.lf;% baseband frequency cutoff 
   settings.reader{i}.demodulation.q     = settings.readerpool.quant{i};
   
   % reader component setup plus for oscillator gitter
   settings.reader{i}.oscillator.fstddev = 1;
   settings.reader{i}.oscillator.astddev = 0;
end 

% tagpool
%     field probe mode: just parameters relevant for the channel
if settings.specials.probemode
   headline('\nTagpool (Probes)');
   critwarn('Setting probe input carrier frequency to frequency of closest reader.');
   for i = 1 : settings.tagpool.n
      % copy from standardsettings
      settings.tag{i} = stdsettings.tag;
      % set carrier frequency to fc of closest reader (warning above)
      % ... this should be considered TEMPORARY (until tag rx path is ultrawideband)
      [max_r, ind_r]  = min(get_distances(settings.readerpool.pos, settings.tagpool.pos{i}));
      settings.tag{i}.modulation.fc   = settings.readerpool.fc{ind_r};
      settings.tag{i}.demodulation.fc = settings.readerpool.fc{ind_r};
   end
   
%     normal operation
else
   headline('\nTagpool');
   critwarn('Settings tag input carrier frequency to frequency of closest reader.');
   for i = 1 : settings.tagpool.n
      % copy from standardsettings
      settings.tag{i} = stdsettings.tag;
      % individual setup
      settings.tag{i}.id.rn16             = settings.tagpool.rn16{i};
      [settings.tag{i}.adsm.as, settings.tag{i}.adsm.ds] = adstate('settings->state', adsm{i},...
         settings.tagpool.cat{i}, settings.tagpool.rat{i}, settings.tagpool.enr{i}, settings.tagpool.fsr{i});
      settings.tag{i}.modulation.charfile = ...
         sprintf('tagchar_modulator_%s-sim_a%03i_d%03i', settings.tagpool.modcharid{i}, ...
         settings.tag{i}.adsm.as, settings.tag{i}.adsm.ds); % filename: ID, assembly state, detuning state
      settings.tag{i}.modulation.t0       = settings.tagpool.t0{i};
      settings.tag{i}.modulation.pwrchar = settings.tagpool.pwrchar{i};
      
      % set carrier frequency to fc of closest reader (warning above)
      % ... this should be considered TEMPORARY (until tag rx path is ultrawideband)
      [max_r, ind_r] = min(get_distances(settings.readerpool.pos, settings.tagpool.pos{i}));
      settings.tag{i}.modulation.fc   = settings.readerpool.fc{ind_r};
      settings.tag{i}.demodulation.fc = settings.readerpool.fc{ind_r};
      
      % initialize tag (complete and sanitize settings)
      settings.tag{i} = tag_main('initialize', settings.tag{i});
   end
end 


% sensatagpool
headline('\nSensatagpool');
critwarn('Setting sensatag input carrier frequency to frequency of closer reader');
for i = 1 : settings.sensatagpool.n
    % copy from standardsettings
    settings.sensatag{i} = stdsettings.sensatag;
    % individual setup
    settings.sensatag{i}.id.rn16   = settings.sensatagpool.rn16{i};
    [settings.sensatag{i}.adsm.as, settings.sensatag{i}.adsm.ds] = adstate('settings->state',sadsm{i},...
        settings.sensatagpool.cat{i}, settings.sensatagpool.rat{i}, settings.sensatagpool.enr{i},...
        settings.sensatagpool.fsr{i});
    settings.sensatag{i}.modulation.charfile = ...
        sprintf('tagchar_modulator_%s-sim_a%03i_d%03i', settings.sensatagpool.modcharid{i}, ...
         settings.sensatag{i}.adsm.as, settings.sensatag{i}.adsm.ds); % filename: ID, assembly state, detuning state
    settings.sensatag{i}.modulation.t0       = settings.sensatagpool.t0{i};
    
    % set sensatag carrier frequency to fc of closer reader
    [max_rs, ind_rs] = min(get_distances(settings.readerpool.pos, settings.sensatagpool.pos{i}));
    settings.sensatag{i}.modulation.fc = settings.readerpool.fc{ind_rs};
    settings.sensatag{i}.demodulation.fc = settings.readerpool.fc{ind_rs};
    
    % initialize sensatag (complete and sanitize settings)
    settings.sensatag{i} = sensatag_main('initialize', settings.sensatag{i});
end



   
% channels reader <-> tag
headline('\nChannels Reader <-> Tag');
%     create channels
for i = 1 : settings.readerpool.n
   for j = 1 : settings.tagpool.n
         % copy from standardsettings
         %     reader -> tag (special treatment for virtual readers)
         settings.channel_rt{i,j}.probemode = stdsettings.channel.probemode;
         settings.channel_rt{i,j}.c  = stdsettings.channel.c;
         settings.channel_rt{i,j}.fs = stdsettings.channel.fs;
         if settings.readerpool.virt{i}
            settings.channel_rt{i,j}.type            = 'outdoor'; % forced long-range model for virtual readers
            settings.channel_rt{i,j}.ucg             = settings.readerpool.virt_gf{i}; % plus gain factor
            settings.channel_rt{i,j}.virtual.vtx     = true; % transmitter is virtual
            settings.channel_rt{i,j}.virtual.vrx     = false;
            settings.channel_rt{i,j}.virtual.otx     = settings.readerpool.virt{i};
            settings.channel_rt{i,j}.virtual.orx     = NaN;
            settings.channel_rt{i,j}.virtual.dim     = settings.readerpool.virt_dim{i};
            settings.channel_rt{i,j}.virtual.refl    = settings.readerpool.virt_refl{i};
            if ~isempty(settings.readerpool.virt_surf{i})
               settings.channel_rt{i,j}.virtual.surface = settings.readerpool.virt_surf{i};
               settings.channel_rt{i,j}.virtual.invsurf = settings.readerpool.virt_inv_surf{i};
            end
         else
            settings.channel_rt{i,j}.type         = stdsettings.channel.type;
            settings.channel_rt{i,j}.virtual.vtx  = false; % transmitter is real
            settings.channel_rt{i,j}.virtual.vrx  = false;
            settings.channel_rt{i,j}.virtual.otx  = NaN;
            settings.channel_rt{i,j}.virtual.orx  = NaN;
            settings.channel_rt{i,j}.virtual.dim  = NaN;
            settings.channel_rt{i,j}.virtual.refl = NaN;
         end
         settings.channel_rt{i,j}.largescale  = stdsettings.channel.largescale;
         if settings.channel_global.surf_on
            settings.channel_rt{i,j}.surfaces = stdsettings.channel.surfaces;
         end
         settings.channel_rt{i,j}.smallscale  = stdsettings.channel.smallscale;
         settings.channel_rt{i,j}.noise       = stdsettings.channel.noise;
         %     tag -> reader (special treatment for virtual readers)
         settings.channel_tr{j,i}.probemode = stdsettings.channel.probemode;
         settings.channel_tr{j,i}.c  = stdsettings.channel.c;
         settings.channel_tr{j,i}.fs = stdsettings.channel.fs;
         if settings.readerpool.virt{i}
            settings.channel_tr{j,i}.type            = 'outdoor'; % forced long-range model for virtual readers
            settings.channel_tr{j,i}.ucg             = settings.readerpool.virt_gf{i}; % plus gain factor
            settings.channel_tr{j,i}.virtual.vtx     = false;
            settings.channel_tr{j,i}.virtual.vrx     = true; % receiver is virtual
            settings.channel_tr{j,i}.virtual.otx     = NaN;
            settings.channel_tr{j,i}.virtual.orx     = settings.readerpool.virt{i};
            settings.channel_tr{j,i}.virtual.dim     = settings.readerpool.virt_dim{i};
            settings.channel_tr{j,i}.virtual.refl    = settings.readerpool.virt_refl{i};
            if ~isempty(settings.readerpool.virt_surf{i})
               settings.channel_tr{j,i}.virtual.surface = settings.readerpool.virt_surf{i};
               settings.channel_tr{j,i}.virtual.invsurf = settings.readerpool.virt_inv_surf{i};
            end
         else
            settings.channel_tr{j,i}.type     = stdsettings.channel.type;
            settings.channel_tr{j,i}.virtual.vtx  = false;
            settings.channel_tr{j,i}.virtual.vrx  = false; % receiver is real
            settings.channel_tr{j,i}.virtual.otx  = NaN;
            settings.channel_tr{j,i}.virtual.orx  = NaN;
            settings.channel_tr{j,i}.virtual.dim  = NaN;
            settings.channel_tr{j,i}.virtual.refl = NaN;
         end
         settings.channel_tr{j,i}.largescale  = stdsettings.channel.largescale;
         if settings.channel_global.surf_on
            settings.channel_tr{j,i}.surfaces = stdsettings.channel.surfaces;
         end
         settings.channel_tr{j,i}.smallscale  = stdsettings.channel.smallscale;
         settings.channel_tr{j,i}.noise       = stdsettings.channel.noise;
         % largescale
         settings.channel_rt{i,j}.largescale.f0 = settings.readerpool.fc{i};
         settings.channel_rt{i,j}.largescale.pl = settings.channel_global.plf;
         settings.channel_tr{j,i}.largescale.f0 = settings.readerpool.fc{i};
         settings.channel_tr{j,i}.largescale.pl = settings.channel_global.plf;
         % directivity
         %     reader -> tag
         settings.channel_rt{i,j}.directivity.txant = settings.readerpool.ant{i};
         settings.channel_rt{i,j}.directivity.txrot = settings.readerpool.ant_rot{i};
         settings.channel_rt{i,j}.directivity.rxant = settings.tagpool.ant{j};
         settings.channel_rt{i,j}.directivity.rxrot = settings.tagpool.ant_rot{j};
         %     tag -> reader
         settings.channel_tr{j,i}.directivity.txant = settings.tagpool.ant{j};
         settings.channel_tr{j,i}.directivity.txrot = settings.tagpool.ant_rot{j};
         settings.channel_tr{j,i}.directivity.rxant = settings.readerpool.ant{i};
         settings.channel_tr{j,i}.directivity.rxrot = settings.readerpool.ant_rot{i}; 
         % smallscale
         settings.channel_rt{i,j}.smallscale.bw   = 300; 
         %+...max([0, settings.readerpool.mfcw.fi{i}]) - min([0, settings.readerpool.mfcw.fi{i}]) );
         settings.channel_tr{j,i}.smallscale.bw   = 300; % This is may not correct for small scale bandwidth
         %+...max([0, settings.readerpool.mfcw.fi{i}]) - min([0, settings.readerpool.mfcw.fi{i}]) );
         settings.channel_rt{i,j}.smallscale.fres = 1200000; % frequency resolution
         settings.channel_tr{j,i}.smallscale.fres = 1200000;
         % noise
         settings.channel_rt{i,j}.noise.frxs = settings.tag{j}.clock.fcenter; % receiver is tag
         settings.channel_tr{j,i}.noise.frxs = settings.reader{i}.demodulation.frs; % receiver is reader
   end
end
%     add all distance-dependent settings
settings.channel_rt = channel_newpos(settings.channel_rt, ...
   settings.readerpool.pos(1:settings.readerpool.n), settings.tagpool.pos(1:settings.tagpool.n), stdsettings.channel);
settings.channel_tr = channel_newpos(settings.channel_tr, ...
   settings.tagpool.pos(1:settings.tagpool.n), settings.readerpool.pos(1:settings.readerpool.n), stdsettings.channel);

% channels reader <-> sensatag
headline('\nChannels Reader <-> Sensatag');
%     create channels
for i = 1 : settings.readerpool.n
   for j = 1 : settings.sensatagpool.n
         % copy from standardsettings
         %     reader -> sensatag (special treatment for virtual readers)
         settings.channel_rs{i,j}.probemode = stdsettings.channel.probemode;
         settings.channel_rs{i,j}.c  = stdsettings.channel.c;
         settings.channel_rs{i,j}.fs = stdsettings.channel.fs;
         if settings.readerpool.virt{i}
            settings.channel_rs{i,j}.type            = 'outdoor'; % forced long-range model for virtual readers
            settings.channel_rs{i,j}.ucg             = settings.readerpool.virt_gf{i}; % plus gain factor
            settings.channel_rs{i,j}.virtual.vtx     = true; % transmitter is virtual
            settings.channel_rs{i,j}.virtual.vrx     = false;
            settings.channel_rs{i,j}.virtual.otx     = settings.readerpool.virt{i};
            settings.channel_rs{i,j}.virtual.orx     = NaN;
            settings.channel_rs{i,j}.virtual.dim     = settings.readerpool.virt_dim{i};
            settings.channel_rs{i,j}.virtual.refl    = settings.readerpool.virt_refl{i};
            if ~isempty(settings.readerpool.virt_surf{i})
               settings.channel_rs{i,j}.virtual.surface = settings.readerpool.virt_surf{i};
               settings.channel_rs{i,j}.virtual.invsurf = settings.readerpool.virt_inv_surf{i};
            end
         else
            settings.channel_rs{i,j}.type         = stdsettings.channel.type;
            settings.channel_rs{i,j}.virtual.vtx  = false; % transmitter is real
            settings.channel_rs{i,j}.virtual.vrx  = false;
            settings.channel_rs{i,j}.virtual.otx  = NaN;
            settings.channel_rs{i,j}.virtual.orx  = NaN;
            settings.channel_rs{i,j}.virtual.dim  = NaN;
            settings.channel_rs{i,j}.virtual.refl = NaN;
         end
         settings.channel_rs{i,j}.largescale  = stdsettings.channel.largescale;
         if settings.channel_global.surf_on
            settings.channel_rs{i,j}.surfaces = stdsettings.channel.surfaces;
         end
         settings.channel_rs{i,j}.smallscale  = stdsettings.channel.smallscale;
         settings.channel_rs{i,j}.noise       = stdsettings.channel.noise;
         %     sensatag -> reader (special treatment for virtual readers)
         settings.channel_sr{j,i}.probemode = stdsettings.channel.probemode;
         settings.channel_sr{j,i}.c  = stdsettings.channel.c;
         settings.channel_sr{j,i}.fs = stdsettings.channel.fs;
         if settings.readerpool.virt{i}
            settings.channel_sr{j,i}.type            = 'outdoor'; % forced long-range model for virtual readers
            settings.channel_sr{j,i}.ucg             = settings.readerpool.virt_gf{i}; % plus gain factor
            settings.channel_sr{j,i}.virtual.vtx     = false;
            settings.channel_sr{j,i}.virtual.vrx     = true; % receiver is virtual
            settings.channel_sr{j,i}.virtual.otx     = NaN;
            settings.channel_sr{j,i}.virtual.orx     = settings.readerpool.virt{i};
            settings.channel_sr{j,i}.virtual.dim     = settings.readerpool.virt_dim{i};
            settings.channel_sr{j,i}.virtual.refl    = settings.readerpool.virt_refl{i};
            if ~isempty(settings.readerpool.virt_surf{i})
               settings.channel_sr{j,i}.virtual.surface = settings.readerpool.virt_surf{i};
               settings.channel_sr{j,i}.virtual.invsurf = settings.readerpool.virt_inv_surf{i};
            end
         else
            settings.channel_sr{j,i}.type     = stdsettings.channel.type;
            settings.channel_sr{j,i}.virtual.vtx  = false;
            settings.channel_sr{j,i}.virtual.vrx  = false; % receiver is real
            settings.channel_sr{j,i}.virtual.otx  = NaN;
            settings.channel_sr{j,i}.virtual.orx  = NaN;
            settings.channel_sr{j,i}.virtual.dim  = NaN;
            settings.channel_sr{j,i}.virtual.refl = NaN;
         end
         settings.channel_sr{j,i}.largescale  = stdsettings.channel.largescale;
         if settings.channel_global.surf_on
            settings.channel_sr{j,i}.surfaces = stdsettings.channel.surfaces;
         end
         settings.channel_sr{j,i}.smallscale  = stdsettings.channel.smallscale;
         settings.channel_sr{j,i}.noise       = stdsettings.channel.noise;
         % largescale
         settings.channel_rs{i,j}.largescale.f0 = settings.readerpool.fc{i};
         settings.channel_rs{i,j}.largescale.pl = settings.channel_global.plf;
         settings.channel_sr{j,i}.largescale.f0 = settings.readerpool.fc{i};
         settings.channel_sr{j,i}.largescale.pl = settings.channel_global.plf;
         % directivity
         %     reader -> sensatag
         settings.channel_rs{i,j}.directivity.txant = settings.readerpool.ant{i};
         settings.channel_rs{i,j}.directivity.txrot = settings.readerpool.ant_rot{i};
         settings.channel_rs{i,j}.directivity.rxant = settings.sensatagpool.ant{j};
         settings.channel_rs{i,j}.directivity.rxrot = settings.sensatagpool.ant_rot{j};
         %     sensatag -> reader
         settings.channel_sr{j,i}.directivity.txant = settings.sensatagpool.ant{j};
         settings.channel_sr{j,i}.directivity.txrot = settings.sensatagpool.ant_rot{j};
         settings.channel_sr{j,i}.directivity.rxant = settings.readerpool.ant{i};
         settings.channel_sr{j,i}.directivity.rxrot = settings.readerpool.ant_rot{i}; 
         % smallscale
         settings.channel_rs{i,j}.smallscale.bw   = 300; 
         %+...max([0, settings.readerpool.mfcw.fi{i}]) - min([0, settings.readerpool.mfcw.fi{i}]) );
         settings.channel_sr{j,i}.smallscale.bw   = 300; % This is may not correct for small scale bandwidth
         %+...max([0, settings.readerpool.mfcw.fi{i}]) - min([0, settings.readerpool.mfcw.fi{i}]) );
         settings.channel_rs{i,j}.smallscale.fres = 1200000; % frequency resolution
         settings.channel_sr{j,i}.smallscale.fres = 1200000;
         % noise
         settings.channel_rs{i,j}.noise.frxs = settings.sensatag{j}.clock.fcenter; % receiver is tag
         settings.channel_sr{j,i}.noise.frxs = settings.reader{i}.demodulation.frs; % receiver is reader
   end
end
%     add all distance-dependent settings
settings.channel_rs = channel_newpos(settings.channel_rs,...
    settings.readerpool.pos(1:settings.readerpool.n), settings.sensatagpool.pos(1:settings.sensatagpool.n), stdsettings.channel);
% settings.channel_sr = channel_newpos(settings.channel_sr, ...
%     settings.sensatagpool.pos(1:settings.sensatagpool.n), settings.readerpool.pos(1:settings.readerpool.n), stdsettings.channel);

% channels tag->sensatag
headline('\nChannels tag->sensatag');
%     create channels
for i = 1 : settings.sensatagpool.n
   for j = 1 : settings.tagpool.n
         % copy from standardsettings 
         %     tag -> reader (special treatment for virtual readers)
         settings.channel_ts{j,i}.probemode = stdsettings.channel.probemode;
         settings.channel_ts{j,i}.c  = stdsettings.channel.c;
         settings.channel_ts{j,i}.fs = stdsettings.channel.fs;
         
         settings.channel_ts{j,i}.type         = stdsettings.channel.type;
         settings.channel_ts{j,i}.virtual.vtx  = false; % transmitter is real
         settings.channel_ts{j,i}.virtual.vrx  = false;
         settings.channel_ts{j,i}.virtual.otx  = NaN;
         settings.channel_ts{j,i}.virtual.orx  = NaN;
         settings.channel_ts{j,i}.virtual.dim  = NaN;
         settings.channel_ts{j,i}.virtual.refl = NaN;
         
         settings.channel_ts{j,i}.type     = stdsettings.channel.type;        
         settings.channel_ts{j,i}.largescale  = stdsettings.channel.largescale;
         if settings.channel_global.surf_on
            settings.channel_ts{j,i}.surfaces = stdsettings.channel.surfaces;
         end
         settings.channel_ts{j,i}.smallscale  = stdsettings.channel.smallscale;
         settings.channel_ts{j,i}.noise       = stdsettings.channel.noise;
         % largescale

         settings.channel_ts{j,i}.largescale.f0 = settings.sensatagpool.fc{i};
         settings.channel_ts{j,i}.largescale.pl = settings.channel_global.plf;
         % directivity

         %     tag -> sensatag
         settings.channel_ts{j,i}.directivity.txant = settings.tagpool.ant{j};
         settings.channel_ts{j,i}.directivity.txrot = settings.tagpool.ant_rot{j};
         settings.channel_ts{j,i}.directivity.rxant = settings.sensatagpool.ant{i};
         settings.channel_ts{j,i}.directivity.rxrot = settings.sensatagpool.ant_rot{i}; 
         % smallscale
         settings.channel_ts{j,i}.smallscale.bw   = 300; % This is may not correct for small scale bandwidth
         %+...max([0, settings.readerpool.mfcw.fi{i}]) - min([0, settings.readerpool.mfcw.fi{i}]) );
         settings.channel_ts{j,i}.smallscale.fres = 1200000;
         % noise

         settings.channel_ts{j,i}.noise.frxs = settings.reader{1}.demodulation.frs; % receiver is sensatag
   end
end
%     add all distance-dependent settings
settings.channel_ts = channel_newpos(settings.channel_ts, ...
    settings.tagpool.pos(1:settings.tagpool.n), settings.sensatagpool.pos(1:settings.sensatagpool.n), stdsettings.channel);

% channels reader -> reader (not needed in probe-mode)
if ~settings.specials.probemode
   headline('\nChannels Reader -> Reader');
   %     create channels (
   for i = 1 : settings.readerpool.n
      for j = 1 : settings.readerpool.n_nonvirt
         % copy from standardsettings (special treatment for virtual readers)
         settings.channel_rr{i,j}.probemode = stdsettings.channel.probemode;
         settings.channel_rr{i,j}.c  = stdsettings.channel.c;
         settings.channel_rr{i,j}.fs = stdsettings.channel.fs;
         if settings.readerpool.virt{i}
            settings.channel_rr{i,j}.type = 'outdoor'; % forced long-range model for virtual readers
            settings.channel_rr{i,j}.ucg  = settings.readerpool.virt_gf{i}; % plus gain factor
            settings.channel_rr{i,j}.virtual.vtx  = true;
            settings.channel_rr{i,j}.virtual.otx  = settings.readerpool.virt{i};
            settings.channel_rr{i,j}.virtual.dim  = settings.readerpool.virt_dim{i};
            settings.channel_rr{i,j}.virtual.refl = settings.readerpool.virt_refl{i};
            if ~isempty(settings.readerpool.virt_surf{i})
               settings.channel_rr{i,j}.virtual.surface = settings.readerpool.virt_surf{i};
               settings.channel_rr{i,j}.virtual.invsurf = settings.readerpool.virt_inv_surf{i};
            end
         else
            settings.channel_rr{i,j}.type         = stdsettings.feedback.type;
            settings.channel_rr{i,j}.virtual.vtx  = false;
            settings.channel_rr{i,j}.virtual.otx  = NaN;
            settings.channel_rr{i,j}.virtual.dim  = NaN;
            settings.channel_rr{i,j}.virtual.refl = NaN;
         end
% %          if settings.readerpool.virt{j}
% %             settings.channel_rr{i,j}.virtual.vrx  = true;
% %             settings.channel_rr{i,j}.virtual.orx  = settings.readerpool.virt{j};
% %             settings.channel_rr{i,j}.virtual.dim  = settings.readerpool.virt_dim{j};
% %             settings.channel_rr{i,j}.virtual.refl = settings.readerpool.virt_refl{j};
% %          else
            settings.channel_rr{i,j}.virtual.vrx = false;
            settings.channel_rr{i,j}.virtual.orx = NaN;
% %          end  
         settings.channel_rr{i,j}.smallscale = stdsettings.feedback.smallscale;
         settings.channel_rr{i,j}.noise      = stdsettings.feedback.noise;
         if i == j % direct feedback to identical reader
            settings.channel_rr{i,j}.direct.gain     = 10^(-stdsettings.feedback.fb_att/20);
            settings.channel_rr{i,j}.direct.delay_s  = round(stdsettings.feedback.fb_del*settings.fs);
            settings.channel_rr{i,j}.direct.gain_dir = 1;    
         else % or normal channel to different reader
            % largescale
            settings.channel_rr{i,j}.largescale     = stdsettings.feedback.largescale;
            settings.channel_rr{i,j}.largescale.f0  = settings.readerpool.fc{i};
            settings.channel_rr{i,j}.largescale.pl  = settings.channel_global.plf;
            % reflective surfaces
            if settings.channel_global.surf_on
               settings.channel_rr{i,j}.surfaces = stdsettings.feedback.surfaces;
            end
            % directivity
            settings.channel_rr{i,j}.directivity       = stdsettings.feedback.directivity;
            settings.channel_rr{i,j}.directivity.txant = settings.readerpool.ant{i};
            settings.channel_rr{i,j}.directivity.txrot = settings.readerpool.ant_rot{i};
            settings.channel_rr{i,j}.directivity.rxant = settings.readerpool.ant{j};
            settings.channel_rr{i,j}.directivity.rxrot = settings.readerpool.ant_rot{j};
         end
         % smallscale
            settings.channel_rr{i,j}.smallscale.bw   = 3000000 ;
            settings.channel_rr{i,j}.smallscale.fres = 3000000; % it is not sure this is correct
         % noise
         settings.channel_rr{i,j}.noise.frxs = settings.reader{j}.demodulation.frs;
      end
   end
   %     add all distance-dependent settings
   settings.channel_rr = channel_newpos(settings.channel_rr, ...
      settings.readerpool.pos(1:settings.readerpool.n), settings.readerpool.pos(1:settings.readerpool.n_nonvirt), stdsettings.feedback);
end

% variable cleanup
clear('max_r', 'ind_r'); 

headline('*******************************************************************************************************');
   headline('* Downlink (Query command)');
   % prepare command
   headline('\nPreparing Query Command');
   for i = 1 : settings.readerpool.n_nonvirt
      headline('   Reader %i', i);
      settings.reader{i} = reader_main('prep_query', settings.reader{i});
   end
     % find maximum needed carrier length
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
         headline('   Reader %i', i);
         % set carrier length to maximum needed length
         settings.reader{i} = reader_main('set_maxclen', settings.reader{i});
         % create modulated carrier
         reader_carrier{i} = reader_main('tx_data', settings.reader{i});
      else
         % create dummy
         reader_carrier{i} = [0];
      end
   end
   
   % channel R -> T
   headline('\nChannel Reader -> Tag');
   tag_carrier = channel_main(reader_carrier, settings.channel_rt);
   
   % channel R -> S
   headline('\nChannel Reader -> Sensatag');
   sensatag_carrier = channel_main(reader_carrier, settings.channel_rs);
   
   % demodulate, decode, determine state for tag
   headline('\nDemodulate + Decode, Get Link Setup');
    for i = 1 : settings.tagpool.n
      headline('   Tag %i', i);
      % re-initialize tag (set to unpowered state)
      settings.tag{i} = tag_main('re-initialize', settings.tag{i});
      % receive, check if powered
      temp = tag_main('rx', tag_carrier{i}, settings.tag{i});
      settings.tag{i} = temp.settings; % modifications to vdda
      tag_rxsignal{i} = temp.rxsignal;
      settings.tag{i}.state.powered = temp.powered;
      % set EPC state accordingly ('' or 'ready')
      settings.tag{i} = tag_main('set_epcstate', settings.tag{i});
      % if not powered => not much sense in decoding
      if ~settings.tag{i}.state.powered; continue; end
      % demodulate and decode
      temp = tag_main('rx_data', tag_rxsignal{i}, settings.tag{i});
      tag_decoded{i} = temp.decoded; %NOT NEEDED 
      settings.tag{i}.state.linkinfo = temp.linkinfo;
       % determine state of tag
      settings.tag{i} = tag_main('set_epcstate', settings.tag{i});
      % setup return link (if possible)
      settings.tag{i} = tag_main('query', settings.tag{i});
    end
    
   % demodulate, decode, determine state for sensatag
   headline('\nDemodulate + Decode')
    for i = 1 : 1% settings.sensatagpool.n
        headline('   Sensatag %i', i);
        % receive, filter
        temp = sensatag_main('rx', sensatag_carrier{i}, settings.sensatag{i});
        settings.sensatag{i} = temp.settings;
        sensatag_rxsignal{i} = temp.rxsignal;
        % demodulate and decode 
        temp = sensatag_main('rx_data', sensatag_rxsignal{i}, settings.sensatag{i});
        settings.sensatag{i}.state.linkinfo = temp.linkinfo;
     end
  % free memory (large vectors) and clean up variables
   clear('reader_carrier', 'tag_carrier', 'sensatag_carrier', 'tag_rxsignal','sensatag_rxsignal','temp', 'nfft',...
       'max_clen_s', 'tempsettings');
   
      % check for inactive tags before continuing (all tags need to be initialized here)
   if any( cellfun(@(x) ~x.state.powered, settings.tag) )
      temp = 1 : settings.tagpool.n;
      err('The following tags were not properly initialized: %s(out of tags 1..%i)',...
         sprintf('%i ', temp(cellfun(@(x) ~x.state.powered, settings.tag))), settings.tagpool.n );
      diary off;
   end


% *******************************************************************************************************
% run simulator: prepare uplink
headline('\n');

 headline('*******************************************************************************************************');
   headline('* Prepare Uplink (RN16 reply of active tags and sensatags)');
   % encode and find needed carrier length
   headline('\nEncoding RN16 and Initializing Modulation');
   for i = 1 : settings.tagpool.n
      headline('   Tag %i', i);
      settings.tag{i} = tag_main('phase_can_pre', settings.tag{i});
   end
   
   for i = 1 : 1% settings.sensatagpool.n
       headline('  Sensatag %i', i);
       settings.sensatag{i} = sensatag_main('prep_rn16',settings.sensatag{i});
   end
   
   % find maximum needed carrier length
   max_clen = max(cellfun(@(x) x.modulation.length, settings.tag));
   max_sclen = max(cellfun(@(x) x.modulation.length, settings.sensatag));
   max_clen = max(max_clen,max_sclen);
   %     and distribute to all readers (add field max_clen to settings)
   settings.reader = cellfun(@(x) setfield(x, 'max_clen', max_clen), settings.reader, 'UniformOutput',false); %#ok<SFLD>
   % create carrier for modulation
  headline('\nCreating Carrier for Modulation');
  for i = 1 : settings.readerpool.n_nonvirt
     headline('    Reader %i',i);
      % set length to maximum carrier length
      settings.reader{i} = reader_main('set_maxclen', settings.reader{i});
      % create unmodulated carrier
      reader_carrier_base{i} = reader_main('tx_carrier', settings.reader{i});
  end
  
  LoopInd = 0; 
  phcan_cnt = 0;
  
  while LoopInd < settings.sloop.n
      LoopInd = LoopInd + 1;
      try
          headline('\n');
          headline('*******************************************************************************************************');
          headline('* Looping Positions');
          headline('* %s, run %i of %i (on %s, pid %.0f)', settings.suffix, LoopInd, settings.sloop.n, settings.hostname, settings.pid );
         
          
          % modify position of sensatags
          headline('\nModifying Positions');
          for i = 1 : settings.sensatagpool.n
            settings.sensatagpool.pos{i} = settings.sloop.pos_t{i}(LoopInd, :);
          end
          
          
          settings.channel_rs = channel_newpos(settings.channel_rs,settings.readerpool.pos(1:settings.readerpool.n),...
              settings.sensatagpool.pos(1:settings.sensatagpool.n),stdsettings.channel);
          settings.channel_sr = channel_newpos(settings.channel_sr,settings.sensatagpool.pos(1:settings.sensatagpool.n),...
              settings.readerpool.pos(1:settings.readerpool.n),stdsettings.channel);

          settings.channel_ts = channel_newpos(settings.channel_ts,settings.tagpool.pos(1:settings.tagpool.n),...
              settings.sensatagpool.pos(1:settings.sensatagpool.n),stdsettings.channel);
        
          % select active reader (all other readers are passive)
          for ActRdr = 1 : settings.readerpool.n_nonvirt
          % ... only one active reader
          headline('\n\n****************************************');
          headline('* Active Reader: %i', ActRdr);
          reader_carrier = cell(1, settings.readerpool.n_nonvirt);
          reader_carrier = cellfun(@(x) [], reader_carrier, 'UniformOutput',false);
          reader_carrier{ActRdr} =  reader_carrier_base{ActRdr};
         
          % channel R -> T
          headline('\nChannel Reader -> Tag');
          %     create identical up/downlink channels to/from active reader
          %     ... [???] more elegant implementation without loops
          %     ... clock * rand should be ok even if rand sequence is accidentally nonrandom (identical seeds)
          if settings.readerpool.id_ch{ActRdr}
             for j = 1 : settings.tagpool.n
                settings.channel_rt{ActRdr,j}.smallscale.seed = round(mod(sum(1e7*clock), 2^32-1) * rand);
                settings.channel_tr{j,ActRdr}.smallscale.seed = settings.channel_rt{ActRdr,j}.smallscale.seed;
             end
            
             for j = 1 : settings.sensatagpool.n
                 settings.channel_rs{ActRdr,j}.smallscale.seed = round(mod(sum(1e7*clock), 2^32-1) * rand);
                 settings.channel_sr{j,ActRdr}.smallscale.seed = settings.channel_rs{ActRdr,j}.smallscale.seed;
             end
          end
          %     apply channels
          [tag_carrier, channel_stat] = channel_main(reader_carrier, settings.channel_rt);
          [sensatag_carrier, schannel_stat] = channel_main(reader_carrier, settings.channel_rs);


          clear('reader_carrier');
          % modulate RN16 or just reflect
          headline('\nModulation of RN16 (only active tags)');
          for i = 1 : settings.tagpool.n
             headline('   Tag %i', i);
             tx_temp = tag_main('tx_data_active', tag_carrier{i}, settings.tag{i});
             tag_modcarrier{i} = tx_temp.txsignal; % signal
             settings.tag{i}   = tx_temp.settings; % modifications to vdda
          end
             clear('tx_temp');
%           % channel T -> R
%           headline('\nChannel Tag -> Reader');
%           [reader_modcarrier, channel_stat] = channel_main(tag_modcarrier, settings.channel_tr);
          % get the smallscale seed
          for i = 1: settings.tagpool.n
              for j = 1 : settings.sensatagpool.n
                  settings.channel_ts{i,j}.smallscale.seed = round(mod(sum(1e7*clock), 2^32-1) * rand);
                  settings.channel_st{j,i}.smallscale.seed = settings.channel_ts{i,j}.smallscale.seed;
              end
          end
          % channel T -> S
          headline('\nChannel Tag -> Sensatag');
          [sensatag_modcarrier,schannel_stat] = channel_main(tag_modcarrier, settings.channel_ts);

          % Channel T+R -> S
          headline('\nChannel T+R -> Sensatag');
          sensatag_modcarrier = cellfun(@(a,b) a+b, sensatag_modcarrier, cellfun(@(a,b) a(1:length(b)),...
              sensatag_carrier,sensatag_modcarrier, 'UniformOutput',false),'UniformOutput',false);
         if settings.sensatagpool.n == 2
          sensatag_modcarrier{3} = sensatag_modcarrier{1}+sensatag_modcarrier{2};
         else
          sensatag_modcarrier{3} = sensatag_modcarrier{1};
         end
         

          % demodualte and decode in sensatag
          headline('\nDemodulate + decode');
          for i = 1 : 1 %settings.sensatagpool.n
              headline('   Sensatag %i', i);
              temp = sensatag_main('rx_power',sensatag_modcarrier{3}, settings.sensatag{i});
                   
          end
          
        %  str = ['Sensatag Position is  ' num2str(settings.sensatagpool.pos{1})];
          fid = fopen('read_rate.txt','a');
          fprintf(fid,'Results %s\n',mat2str(temp.demod));
          fclose(fid);
          

          
%           phcan_s1(1) = 0;
%           phcan_s2(1) = 1;
%           
%           for s_cnt = 1 : length(temp.demod)-1
%               phcan_s1(s_cnt+1) = ~phcan_s1(s_cnt);
%               phcan_s2(s_cnt+1) = ~phcan_s2(s_cnt);
%           end
%           
% 
%           if isequal(temp.demod,phcan_s1') || isequal(temp.demod,phcan_s2')
%               phcan_cnt = phcan_cnt;
%           else
%               fid = fopen('phcan_rec.txt','a');
%               fprintf(fid,'\nPhase Cancellation Happened: %s.\t',str);
%               fprintf(fid,'Results %d\n',LoopInd);
%               fclose(fid);
%               
%               phcan_cnt = phcan_cnt + 1;
%           end
          
%           gap1 =  am1_1-am1_2
%           gap1_a = am1_1_a-am1_2_a
%           %gap2 =  max(sensatag_modcarrier{2}(0.4e5:1.6e5))-max(sensatag_modcarrier{2}(2e5:3e5));
%           if abs(gap1) > 0.0005
%               if abs(gap1_a) <= 0.0002
%                   fid = fopen ('phcan_rec.txt','a');
%                   fprintf(fid,'\nPhase Cancellation Happened:( %s )\t',str);
%                   fprintf(fid,'\nThe decoding: %s\n',mat2str(temp.demod));
%                   fclose(fid);
%                   phcan_cnt = phcan_cnt + 1;
%               end
%           end

%%% record backscattering level
%                   fid = fopen('phcan_record.txt','a');
%                   fprintf(fid,'\nSC1: carrier is %d, state1 is %d, state2 is %d, the gap is %d',am1_sc,am1_1,am1_2, gap1);
%                   fprintf(fid,'\n The modulation gap is %s',gap1_a);
%                   fprintf(fid,'\nThe decoding: %s\n',mat2str(temp.demod));
%                   fprintf(fid,'\n the position is %s\n\n',str);
%                  % fprintf(fid,'\nSC2: carrier is %d, state1 is %d, state2 is %d',am2_sc,am2_1,am2_2);
%                   fclose(fid);
%                   
%                   fid = fopen('amp_between.txt','a');
%                   fprintf(fid,'\n%s',gap1_a);
%                   fclose(fid);
%%% end: record backscattering level

%           phcan_s1 = zeros(1,length(temp.demod));
%           phcan_s2 = ones(1,length(temp.demod));          
%             if isequal(temp.demod,phcan_s1') || isequal(temp.demod,phcan_s2')
%                 if gap1 > 0.0010
%                   fid = fopen('phcan_rec.txt','a');
%                   fprintf(fid,'\nPhase Cancellation Happened: %s.\t',str);
%                   fprintf(fid,'Results %d\n',phcan_cnt);
%                   fprintf(fid,'\nSC1: carrier is %d, state1 is %d, state2 is %d, the gap is %d',am1_sc,am1_1,am1_2, gap1);
%                  % fprintf(fid,'\nSC2: carrier is %d, state1 is %d, state2 is %d',am2_sc,am2_1,am2_2);
%                   fclose(fid);
%                   phcan_cnt = phcan_cnt + 1;
%                 else
%                   phcan_cnt = phcan_cnt + 1;
%                 end
%             else 
%                   phcan_cnt = phcan_cnt;
%             end
%          headline('The phase cancellation rate is %d out of %d distance is %d',phcan_cnt,LoopInd,settings.loop.pos_t{1}(1)*sqrt(2));
%           figure(1);
%           stairs(temp.demod);
%           ylim([-0.5,1.5]);
%           str = ['Current Sensatag Position is  ' num2str(settings.sensatagpool.pos{1})];
%           title(str);
%               
%           %save(['C:\project\phase_can\simulator\sensatag_main\phasecan\results',num2str(LoopInd)],'figure(1)');
%           
%           filestr = ['result' num2str(LoopInd)];
%           print(gcf,'-djpeg',filestr);
%           
%           figure(2);
%           plot(temp.signal);
%           title(str);
%           filestr = ['result_signal' num2str(LoopInd)];
%           print(gcf,'-djpeg',filestr);
          end
          
   catch ME
      % automatic mode: option to repeat the last loop (if error can be removed)
      if settings.err_rep
         disp(sprintf('\n\n********* Simulation halted due to an error. Press a button to restart previous loop, [Strg]+[C] to abort. *********\n'));
         % try to send an email
         try
            disp('   Trying to send an eMail...');
            send_email('User interaction required (caught an error).',...
               sprintf('The following error was caught by %s on %s; the simulation has been halted.\n\n******************************\n\n%s',...
               mfilename, datestr(now, 0), getReport(ME, 'extended', 'hyperlinks','off')));
         catch %#ok<CTCH>
            disp('      ... nope, didn''t work. This seems to be serious.');
         end
         disp(sprintf('\n*********\n%s', getReport(ME, 'extended', 'hyperlinks','off')));
         disp(sprintf('********* Simulation halted due to an error. Press a button to restart previous loop, [Strg]+[C] to abort. *********'));
         % pause simulation and restart last loop if requested
         pause;
         LoopInd = LoopInd - 1;
      % "debug" mode: throw the error, enter debug mode if activated
      else
         rethrow(ME);
      end
   end 
      



end

