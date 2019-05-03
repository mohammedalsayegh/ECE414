function out = getallspecs(varargin)
%GETALLSPECS Get all common control system specifications.
% GETALLSPECS(G,D) returns all common control system specifications for a
% unity feedback control system having a plant transfer function G and a
% cascade controller having transfer function D.
%
% GETALLSPECS(G,F,H) returns all common control system specifications for a
% two parameter feedback control system having a plant transfer function G,
% a feedforward controller F, and a feedback controller H. 
%
%
% The output is a structure containing the following fields:
%
% RiseTime --> Time it takes the system to rise from 10% to 90% of the
%              final value.
% SettlingTime --> Time it takes the system to settle to within 2% of the
%                  final value.
% SettlintMin --> Minimum value of output once the response has risen.
% SettlingMax --> Maximum value of output once the response has risen.
% Overshoot --> Percent overshoot relative to the final value.
% Undershoot --> Percent undershoot
% Peak --> Peak absolute value of output.
% PeakTime --> Time where Peak occurs.
% Umax --> Peak control effort for unit step input
%
% EssStep --> Steady state error for a unit step input
% EssRamp --> Steady state error for a unit ramp input
%
% Gm --> Gain margin in dB
% PM --> Phase margin in degrees
% Wcg --> Gain crossover frequency in rad/s
% Wcp --> Phase crossover frequency in rad/s
% Vm --> Vector margin
% Wvm --> Vector margin frequency
%
% Smax --> Peak sensitivity

% D.C. Hanselman, 2019/03/06

% input error checking
for k = 1:nargin
   X = varargin{k}; % grab k-th input argument
   if ~ismember(class(X),{'zpk','tf','pid'})
      error('Input must be a transfer function object.')
   end
   if ~issiso(X)
      error('Input must be an SISO system.')
   end
   if ~isreal(X)
      error('Input must have real coefficients.')
   end
end
if nargin==2
   G = varargin{1};
   D = varargin{2};
   L = D*G;
   T = feedback(L,1);
elseif nargin==3
   G = varargin{1};
   F = varargin{2};
   H = varargin{3};
   L = G*H;
   T = minreal(F*feedback(G,H));
else
   error('One or Two input transfer functions required.')
end

% get step response specs
out = stepinfo(T);

% get control effort transfer function
Tu = minreal(T/G);
if ~isproper(Tu)
   out.Umax = inf;
else
   s = stepinfo(Tu);
   out.Umax = s.Peak;
end

% get steady state errors from the closed loop transfer function
out.EssStep = 1 - round(dcgain(T),8);
if out.EssStep == 0
   [N,D] = tfdata(T);
   num = N{1};
   den = D{1};
   out.EssRamp = (num(end-1)-den(end-1))/den(end);
else
   out.EssRamp = inf;
end

% get frequency response specs
warning('off','Control:analysis:MarginUnstable');
[Gm,Pm,Wcg,Wcp] = margin(L);   % get traditional margins
out.Gm = 20*log10(Gm);         % convert Gain margin to dB
out.Pm = Pm;
out.Wcg = Wcg;
out.Wcp = Wcp;
warning('on','Control:analysis:MarginUnstable');

% get vector margin and peak sensitivity
[Smax,Wvm] = getPeakGain(minreal(1/(1+L)));% peak sensitivity
out.Vm = 1/Smax;
out.Wvm = Wvm;

out.Smax = Smax;
