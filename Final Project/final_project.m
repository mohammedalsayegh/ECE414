%% ECE 414 - Final Project
% Name: Mohammed H. Al-Sayegh

% The parts you have to build the system are: 
% Gearbox inertia
Jg = 6.2e-6;	% kg-m2 (as seen from the motor)
% Angular sensor inertia
Js = 1.4e-7;	% kg-m2 
% Voltage amplifier gain
Gv = 5;         % V/V (control signal gain to motor)

%%
% Use engineering judgment and choose one of the following motors: 
% Motor #1 
% Motor inertia 
Jm = 5.0e-5;	%kg-m2 
% Motor viscous friction 
Bm = 3.0e-6;	%Nm-s/rad 
% Motor Torque constant
KT = 0.225;     %±10% Nm/A 
% Motor Resistance
R = 8;          %ohm 
% Motor Inductance
L = 25;         %mH

%%
% Motor #2
% Motor inertia 
Jm = 5.0e-5;	%kg-m2 
% Motor viscous friction 
Bm = 3.0e-6;	%Nm-s/rad 
% Motor Torque constant
KT = 0.175;     %±10% Nm/A 
% Motor Resistance
R = 6;          %ohm 
% Motor Inductance
L = 16;         %mH

%%
% Motor #3
% Motor inertia 
Jm = 5.0e-5;	%kg-m2 
% Motor viscous friction 
Bm = 3.0e-6;	%Nm-s/rad 
% Motor Torque constant
KT = 0.125;     %±10% Nm/A 
% Motor Resistance
R = 4;          %ohm 
% Motor Inductance
L = 7.5;         %mH

%%
% Motor #4
% Motor inertia 
Jm = 5.0e-5;	%kg-m2 
% Motor viscous friction 
Bm = 3.0e-6;	%Nm-s/rad 
% Motor Torque constant
KT = 0.275;     %±10% Nm/A 
% Motor Resistance
R = 12;         %ohm 
% Motor Inductance
L = 32;         %mH