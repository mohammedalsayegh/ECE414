%%
% ECE 414 - Take Home test, plot the root locus for all Alpha values 
% Name: Mohammed Al-Sayegh

close all;
clear;
clc;

Month = 12;
Day = 28;

for Alpha = 1:100
    hold on;
    Alpha_tf = ece414planttf(Month,Day,Alpha);
    rlocus(Alpha_tf);
    grid on
    axis([-40 0 -25 25]);
end

% Allocate memory for poles and gain
p = zeros(100,3);
k = zeros(100,1);

% Loop for every Alpha value from 1 to 100 for 'ece414planttf'
% pass value of the gain and poles location in plane ikn array of 'p'
for (i = 1:100)
    G = ece414planttf(Month,Day,i);
    p(i,:) = pole(G)';
    k(i) = G.K;
end

% find the value of the mean of poles and gain of Alpha value from 1 to 100
pnom = mean(p);
knom = mean(k);

% The norminal transfer function 
% sys = zpk(0,pnom,knom)
s = tf('s');
a = -pnom(1);
b = -pnom(2);
c = -pnom(3);
G = (knom)/((s+a)*(s+b)*(s+c))
G = zpk(G);

rlocus(G);
hold off;