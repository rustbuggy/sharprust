format compact
clc
clear all
close all

%l = 15
%r = 18
%fl = 59.21
%fr = 16.86
%mc_x = -6.73
%mc_y = 13.45

l = 33.94
r = 18
fl = 19
fr = 0.05
%mc_x = -7.34
%mc_y = 3.37

a1 = sqrt(1/2) * fr
a2 = sqrt(1/2) * fl
a3 = r
a4 = l

mc_x = (a1 + a3 - a2 - a4) / 4
mc_y = (a1 + a2) / 4

dist = sqrt(mc_x^2 + mc_y^2)
angle = 57.2957795 * atan2(mc_y, mc_x)

%{
a1 = sqrt(1/2) * fr
a2 = sqrt(1/2) * fl
a3 = r
a4 = l

max_x_neg = min(a2, a4)
min_x_pos = min(a1, a3)
min_y_pos = min(a1, a2)

mc_x = 90 + (min_x_pos - max_x_neg) / 2
mc_y = 190 + min_y_pos / 2
%}

%{
plot(0, 0, '+');
hold on;
x = [a1, -a2, a3, -a4];
y = [a1, a2, 0, 0];
plot(x, y, 'o');
plot(mc_x, mc_y, 'r+');
%}
