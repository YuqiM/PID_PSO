%program to edit the size of the various text areas on plots
%need them all to be the same for the thesis!

%set(h, 'FontAngle',  get(ax, 'FontAngle'), ...
%        'FontName',   get(ax, 'FontName'), ...
%        'FontSize',   get(ax, 'FontSize'), ...
%        'FontWeight', get(ax, 'FontWeight'), ...
%        'string',     string, varargin{:});

%get the handle of the current axes; need to have the figure of interest active!
set(groot,'defaultFigureColor','w')
a=gca;
set(groot,'defaultFigureColor','w')
a=gca;
%turn on the box and grid
box on;
grid on;

% first set the properties of the text for the axes
set(a,'fontsize',20);
%works!

%edit the y label
h = get(a,'ylabel');
set(h,'fontsize',20);
%works!

%edit the x label
h = get(a,'xlabel');
set(h,'fontsize',20);
%works!

%set all the line widths
h=get(a,'children');
set(h,'LineWidth',2.0);
set(a,'linewidth',2.0);
%h = get(a,'LineWidth');
%set(h,2.0);