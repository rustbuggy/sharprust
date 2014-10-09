format compact
clc
clear all
close all

l = 15
r = 37
fl = 35
fr = 16
%mc_x 4 mc_y 8

a1 = sqrt(1/2) * fr
a2 = sqrt(1/2) * fl
a3 = r
a4 = l

max_x_neg = min(a2, a4)
min_x_pos = min(a1, a3)
min_y_pos = min(a1, a2)

%mc_x = (a1 + a3 - a2 - a4) / 4
%mc_y = (a1 + a2) / 4
mc_x = 90 + (min_x_pos - max_x_neg) / 2
mc_y = 190 + min_y_pos / 2

plot(0, 0, '+');
hold on;
x = [a1, -a2, a3, -a4];
y = [a1, a2, 0, 0];
plot(x, y, 'o');
plot(mc_x, mc_y, 'r+');