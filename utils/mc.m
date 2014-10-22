format compact
clc
clear all
close all

%{
l = 70
fl = 100
f = 20
fr = 20
r = 20

s1 = l + fl
a1 = (180 * l + 135 * fl) / s1
bs = s1
ba = a1

s2 = fl + f
a2 = (135 * fl + 90 * f) / s2
if (bs < s2)
    bs = s2
    ba = a2
end

s3 = f + fr
a3 = (90 * f + 45 * fr) / s3
if (bs < s3)
    bs = s3
    ba = a3
end

s4 = fr + r
a4 = (45 * fr + 0 * r) / s4
if (bs < s4)
    bs = s4
    ba = a4
end

bs
ba
ba2 = (180*l + 135*fl + 90*f + 45*fr + 0*r) / (l+fl+f+fr+r)

a1 = sqrt(1/2) * fr
a2 = sqrt(1/2) * fl
a3 = r
a4 = l
a5 = f

mc_x = 20 * cos(degtorad(ba))
mc_y = 20 * sin(degtorad(ba))

plot(0, 0, '+');
hold on;
x = [a1, -a2, a3, -a4, 0];
y = [a1, a2, 0, 0, a5];
plot(x, y, 'o');
plot(mc_x, mc_y, 'r+');
%}

%{
L = 10
x1 = 0; y1 = 0;
x2 = 10; y2 = 10;
x3 = x2 + L; y3 = y2;

%z = 2*(x1*y2 + x3*y1 + x2*y3 - (x3*y2 + x2*y1 + x1*y3))
%h = ((x1^2+y1^2)*y2 + (x3^2+y3^2)*y1 + (x2^2+y2^2)*y3 - ((x3^2+y3^2)*y2 + (x2^2+y2^2)*y1 + (x1^2+y1^2)*y3)) / z

mr = (y2-y1)/(x2-x1)
mt = (y3-y2)/(x3-x2)

x = (mr*mt*(y3-y1) + mr*(x2+x3) - mt*(x1+x2)) / 2*(mr-mt)
y = -1/mr*(x-(x1+x2)/2) + (y1+y2)/2
R = sqrt(x^2 + y^2)

alpha1 = radtodeg(atan(L/R))
alpha2 = radtodeg(atan2(y2, x2))
alpha1 / alpha2
%}

for i = 0:0
    %l = randi([0, 100]);
    %fl = randi([0, 100]);
    %f = randi([0, 100]);
    %fr = randi([0, 100]);
    %r = randi([0, 100]);
    l = 21.213203435596425732025330863145;
    fl = 30;
    f = 100;
    fr = 40;
    r = 28.284271247461900976033774484194;

    a1 = sqrt(1/2) * fr;
    a2 = sqrt(1/2) * fl;
    a3 = r;
    a4 = l;
    a5 = f;

    mc_x = (a1^2 + a3^2 - a2^2 - a4^2) / (a1+a2+a3+a4+a5);
    mc_y = (a1^2 + a2^2 + a5^2) / (a1+a2+a3+a4+a5);

    dist = sqrt(mc_x^2 + mc_y^2);
    angle = 57.2957795 * atan2(mc_y, mc_x);
    steer = 90 - (angle - 90);

    fprintf('l %.2f fl %.2f f %.2f fr %.2f r %.2f mc_x %.2f mc_y %.2f dist %.2f angle %.2f steer %.2f\n', l, fl, f, fr, r, mc_x, mc_y, dist, angle, steer);

    %
    plot(0, 0, '+');
    hold on;
    x = [a1, -a2, a3, -a4, 0];
    y = [a1, a2, 0, 0, a5];
    plot(x, y, 'o');
    plot(mc_x, mc_y, 'r+');
    %
end
