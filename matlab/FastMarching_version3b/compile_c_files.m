% This script will compile all the C files of the registration methods
cd('functions');
files=dir('*.c');
clear msfm2d
% mex('msfm2d.c');
% add -v to turn verbose mode back on
mex -compatibleArrayDims GCC='/usr/bin/g++-6' msfm2d.c
clear msfm3d
% mex('msfm3d.c');
mex -compatibleArrayDims GCC='/usr/bin/g++-6' msfm3d.c
cd('..');

cd('shortestpath');
clear rk4
% mex('rk4.c');
mex -compatibleArrayDims GCC='/usr/bin/g++-6' rk4.c
cd('..')
