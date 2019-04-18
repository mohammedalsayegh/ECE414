%%
% ECE 414 - Take Home test, Norminal transfer function
% Name: Mohammed Al-Sayegh 
clc
clear

% Delcare the Day and month of birth
Month = 12;
Day = 28;

% create dynamic for poles and gain
Pole_Alpha_tf = [];
Gain = [];

% Loop for every Alpha value from 1 to 100 for 'ece414planttf'
% pass value of the gain and poles location in plane ikn array of 'p'
for Alpha = 1:100
    Alpha_tf = ece414planttf(Month,Day,Alpha);
    Pole_tf = pole(Alpha_tf);
    Pole_Alpha_tf = [Pole_Alpha_tf,Pole_tf];
    [Z,K]=zero(Alpha_tf);
    Gain = [Gain, K]
end

% Create array of real and imaginary coordination
real_G = real(Pole_Alpha_tf);
imag_G = imag(Pole_Alpha_tf);

% Create array of mean value of all real and imaginary coordination
Norminal_tf_real = [mean(real_G(1,:)),mean(real_G(2,:)), mean(real_G(3,:))];
Norminal_tf_imaginary = [mean(imag_G(1,:)),mean(imag_G(2,:)), mean(imag_G(3,:))];

% Create subplot and plot poles of every Alpha and mean value of  Alpha
ax1 = subplot(2,1,1);
hold on
title('Plant Poles and Norminal Pole Locations')
scatter(real_G(1,:),imag_G(1,:));
scatter(real_G(2,:),imag_G(2,:));
scatter(real_G(3,:),imag_G(3,:));
scatter(Norminal_tf_real,Norminal_tf_imaginary);
xlabel('Real  Axis')
ylabel('Imaginary Axis')
hold off

% Create subplot and plot gain of every Alpha and mean value as line
ax2 = subplot(2,1,2);
hold on;
title('Plant Gain Variations and Norminal Gain')
plot([1:100],Gain)
xlabel('Plant Number')
ylabel('Plant Gain')
hold off;