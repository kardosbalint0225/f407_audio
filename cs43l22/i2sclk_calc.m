clc;
clear all;
close all;

%% I2SCLK calc

%PLLI2S      = [1e+6, 2e+6]';        % PLLI2S source = 1 MHz, 2 MHz
PLLI2S      = 1e+6;        % PLLI2S source = 1 MHz
PLLI2S_N    = (50:432)';            % *N = 50..432
PLLI2S_R    = (2:7)';               % /R = 2..7
Fs          = [8000, 11025, 12000, ... 
                16000, 22050, 24000, ...
                32000, 44100, 48000, ...
                88200, 96000]';     % Fs supported sampling rates

fPLLI2S = [];
fs      = [];
R       = [];
N       = [];
I2SxCLK = [];
div     = [];
fs_c    = [];
e       = zeros(length(PLLI2S_N), length(PLLI2S_R));

diary i2sclk_calc.log

fprintf('+----------+------------+---------------+-----+-----+---------------+-----+\n');
fprintf('|    Fs    |   PLLI2S   |    I2SxCLK    |  N  |  R  |       e       | DIV |\n');
fprintf('+----------+------------+---------------+-----+-----+---------------+-----+\n');

for f = 1 : length(Fs)     
    for pllsrc = 1 : length(PLLI2S)          
        for c = 1 : length(PLLI2S_R)             
           for r = 1 : length(PLLI2S_N)
               
               fPLLI2S  = PLLI2S(pllsrc);
               fs       = Fs(f);
               R        = PLLI2S_R(c);
               N        = PLLI2S_N(r);
                
               I2SxCLK  = (N/R) * fPLLI2S;
               div      = I2SxCLK / (256*fs);
               fs_c     = I2SxCLK / (256*round(div));
               
               if (I2SxCLK > 192e+6) || (fPLLI2S*N < 100e+6) || (fPLLI2S*N > 432e+6) || (round(div) < 4)
                   err  = 9000;
               else
                   err  = ((fs_c/fs)-1)*100; 
               end
               
               e(r, c)  = abs(err);
               
           end
        end
                
        [Np, Rp] = find(e == 0);
        if isempty(Np) || isempty(Rp)
           [Np, Rp] = find(e == min(e(:))); 
        end
        
        for i = 1 : length(Np)                
               
           fPLLI2S  = PLLI2S(pllsrc);
           fs       = Fs(f);
           R        = PLLI2S_R(Rp(i));
           N        = PLLI2S_N(Np(i));
           I2SxCLK  = (N/R) * fPLLI2S;
           div      = I2SxCLK / (256*fs);
           fs_c     = I2SxCLK / (256*round(div));
           err      = ((fs_c/fs)-1)*100;
           fprintf('| %5d Hz | %6d MHz | %9.4f MHz | %3d |  %1d  | %7.9f %% | %3d |\n', fs, fPLLI2S/1e+6, I2SxCLK/1e+6, N, R, abs(err), round(div));
               
        end            
    end
    
    fprintf('|          |            |               |     |     |               |     |\n');
    fprintf('+----------+------------+---------------+-----+-----+---------------+-----+\n');
    
end

diary off


