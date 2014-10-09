format longG
format compact
clc

% NB! will overwrite lookups.c !!!

% Sharp IR sensor 10..80 cm
lookup = zeros(1,1024);
for i = 1:1024
    if i > 80
        voltFromRaw = (i - 1) * (4999 / 1023) + 1;
        cm = 27.728 * ((voltFromRaw / 1000) ^ (-1.2045));
        %if ((cm < 6) || (cm > 80))
        %    lookup(1,i) = 100; % to integer
        %else
        lookup(1,i) = fix(cm); % to integer
        %end
    else
        lookup(1,i) = fix(0.0375 * (i-1));
    end
end

% create the lookup source file
fid = fopen('lookups.cpp', 'w');
fprintf(fid, '#include "lookups.h"\n\n');

fprintf(fid, 'const uint8_t irLookup[1024] = {\n');
for y = 1:64
    for x = 1:16
        ind = (y - 1) * 16 + x;
        fprintf(fid, '%3d, ', lookup(1, ind));
    end
    fprintf(fid, '\n');
end
fprintf(fid, '};\n\n');

fclose(fid);