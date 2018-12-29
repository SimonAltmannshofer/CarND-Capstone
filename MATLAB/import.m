%% Import data from text file.
% Script for importing data from the following text file:
%
%    D:\MATLAB\udacity\manual_driving_log_6.csv
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2018/12/24 18:10:08

%% Initialize variables.
filename = 'D:\MATLAB\udacity\manual_driving_log_6.csv';
delimiter = ',';
startRow = 2;

%% Format for each line of text:
%   column1: double (%f)
%	column2: double (%f)
%   column3: double (%f)
%	column4: double (%f)
%   column5: double (%f)
%	column6: double (%f)
%   column7: double (%f)
%	column8: double (%f)
%   column9: double (%f)
%	column10: double (%f)
%   column11: double (%f)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Allocate imported array to column variable names
time = dataArray{:, 1};
x = dataArray{:, 2};
y = dataArray{:, 3};
v = dataArray{:, 4};
v_d = dataArray{:, 5};
a = dataArray{:, 6};
v_des = dataArray{:, 7};
a_des = dataArray{:, 8};
throttle = dataArray{:, 9};
brake = dataArray{:, 10};
steer = dataArray{:, 11};


%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans;