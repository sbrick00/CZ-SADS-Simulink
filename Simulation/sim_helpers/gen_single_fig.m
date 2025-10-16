function figOut = gen_single_fig()
set(groot, 'defaultTextInterpreter','latex', ...
           'defaultAxesTickLabelInterpreter','latex', ...
           'defaultLegendInterpreter','latex');

W = 7.5;
H = 4.9;

PAL = [ ...
  0.000 0.447 0.741;  % strong blue
  0.850 0.325 0.098;  % orange
  0.13, 0.54, 0.13;  % green
  0.494 0.184 0.556;  % purple
  0.301 0.745 0.933;  % cyan (still visible on projectors)
  0.635 0.078 0.184;  % red
  0.200 0.200 0.200]; % dark gray/black for contrast
set(groot,'defaultAxesColorOrder',PAL);
set(groot,'defaultAxesLineStyleOrder',{'-','--',':','-.'});


% Create figure at exact size
figOut = figure('Units','inches','Position',[5 -1 W H], 'Color','w');
ax = gca;
ax.TickLabelInterpreter = 'latex';
% ax.YAxis.Exponent = -2; 
pad = 0.015;   % inches-ish; small but safe for labels/ticks
ax.LooseInset = max(ax.TightInset, pad*[1 1 1 1]);
end

