clc;
clear all;
close all;

N = 256;
n = (0 : N-1);
A = 2^15;
B = 0;
s = A*sin(2*pi*n/N) + B;
s = int16(s);

l = s';
r = s';
a = [l r]';
a = a(:);
a = [a; a; a; a;];

for i = 1 : 8 : length(a)
    fprintf('%6d, %6d, %6d, %6d, %6d, %6d, %6d, %6d,\n',... 
              a(i), a(i+1), a(i+2), a(i+3), a(i+4), a(i+5), a(i+6), a(i+7));
end


