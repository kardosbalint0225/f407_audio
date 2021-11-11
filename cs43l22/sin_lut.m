clc;
clear all;
close all;

N = 1024;
n = (0 : N-1);
a = 2^15;
b = 0;
s = a*sin(2*pi*n/N) + b;
s = int16(s);

for i = 1 : 8 : N-8
    fprintf('%6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d,\n',... 
              s(i),  s(i+1), s(i+2), s(i+3), s(i+4), s(i+5), s(i+6), s(i+7));
end


