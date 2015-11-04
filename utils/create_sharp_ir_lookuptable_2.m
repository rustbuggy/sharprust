format longG
format compact
clc

% NB! will overwrite lookups.c !!!

% Sharp IR sensor 10..80 cm
lookup = zeros(1,1024);
for i = 1:1024
        sensorValue = i - 1;

        cm = 187754 * sensorValue ^ -1.51;

        lookup(1,i) = fix(cm); % to integer
        if lookup(1,i) > 150
            lookup(1,i) = 150;
        end
end

% create the lookup source file
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
