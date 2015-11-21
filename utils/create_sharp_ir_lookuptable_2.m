format longG
format compact
clc

% NB! will overwrite lookups.c !!!

linear = [
    143 150
    153 125
    164 100
    174 85
    184 70
    201 50
    297 30
    399 20
    726 10 ];

%{
    % front left
    950 10
    560 20
    415 30
    345 40
    300 50
    275 60
    250 70
    235 80
    220 90
    210 100
    200 110
    d = 372251 * x^(-1.55)
    
    % front right
    895 10
    520 20
    390 30
    320 40
    270 50
    240 60
    220 70
    205 80
    195 90
    180 100
    175 110
    d = 191405 * x^(-1.46)
    
    % left
    980 10
    555 20
    405 30
    335 40
    285 50
    255 60
    230 70
    220 80
    205 90
    200 100
    190 110
    d = 202252 * x^(-1.46)
    
    % right
    975 10
    580 20
    435 30
    360 40
    315 50
    285 60
    265 70
    245 80
    235 90
    225 100
    220 110
    d = 520986 * x^(-1.59)
%}

% Sharp IR sensor 10..80 cm
lookup = zeros(1,1024);
for i = 1:1024
        sensorValue = i - 1;
        voltage = sensorValue * (5 / 1023);
        
        %{
        if sensorValue < 143
            cm = 150
        elseif sensorValue >= 143 && sensorValue < 153
            cm = fix((125-150)/(153-143) * (sensorValue - 143) + 150)
        elseif sensorValue >= 153 && sensorValue < 164
            cm = fix((100-125)/(164-153) * (sensorValue - 153) + 125)
        elseif sensorValue >= 164 && sensorValue < 174
            cm = fix((85-100)/(174-164) * (sensorValue - 164) + 100)
        elseif sensorValue >= 174 && sensorValue < 184
            cm = fix((70-85)/(184-174) * (sensorValue - 174) + 85)
        elseif sensorValue >= 184 && sensorValue < 201
            cm = fix((50-70)/(201-184) * (sensorValue - 184) + 70)
        elseif sensorValue >= 201 && sensorValue < 297
            cm = fix((30-50)/(297-201) * (sensorValue - 201) + 50)
        elseif sensorValue >= 297 && sensorValue < 399
            cm = fix((20-30)/(399-297) * (sensorValue - 297) + 30)
        elseif sensorValue >= 399 && sensorValue < 726
            cm = fix((10-20)/(726-399) * (sensorValue - 399) + 20)
        else
            cm = 10
        end
        %}
        %
        %cm = fix(187754 * sensorValue^(-1.51));
        %cm = fix(372251 * sensorValue^(-1.55)); % front left
        %cm = fix(191405 * sensorValue^(-1.46)); % front right
        %cm = fix(202252 * sensorValue^(-1.46)); % left
        cm = fix(520986 * sensorValue^(-1.59)); % right
        if cm > 100
            cm = 100;
        elseif cm < 10
            cm = 10;
        end
        %
        
        %[sensorValue, voltage, cm];

        lookup(1,i) = fix(cm); % to integer
end

% create the lookup source file
%
fid = fopen('../../common/pololu_test/lookups.cpp', 'w');
fprintf(fid, '#include "lookups.h"\n\n');

fprintf(fid, 'const fixed irLookup[1024] = {\n');
for y = 1:64
    for x = 1:16
        ind = (y - 1) * 16 + x;
        fprintf(fid, '%3d, ', lookup(1, ind));
    end
    fprintf(fid, '\n');
end
fprintf(fid, '};\n\n');

fclose(fid);
%