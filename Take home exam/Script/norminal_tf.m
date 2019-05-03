%%
% ECE 414 - Take Home test, Norminal transfer function
% Name: Mohammed Al-Sayegh

clc
clear

% Delcare the Day and month of birth
Month = 12;
Day = 28;

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

% Create subplot and plot poles of every Alpha and mean value of  Alpha
figure(1);
ax1 = subplot(2,1,1);
hold on;
title('Plant Poles and Norminal Pole Locations')
plot(p,'x','Color','red');
plot(pnom,'*','Color','blue');
axis([-23 0 -23 23]);
xlabel('Real  Axis')
ylabel('Imaginary Axis')
grid on;
hold off

% Create subplot and plot gain of every Alpha and mean value as line
ax2 = subplot(2,1,2);
hold on;
title('Plant Gain Variations and Norminal Gain')
plot(k,'Color','blue');
yline(knom);
xlabel('Plant Number')
ylabel('Plant Gain')
hold off;


% The norminal transfer function 
G = zpk([],pnom,knom)

% Plant plot with range allowed to use for controller
figure(22);
hold on;
title('Plant Poles and Norminal Pole Locations')
plot(p,'x','Color','red');
plot(pnom,'*','Color','blue');
axis([-23 0 -23 23]);
xlabel('Real  Axis')
ylabel('Imaginary Axis')
rlocus(G)
grid on;
hold off;

specs_table(G);