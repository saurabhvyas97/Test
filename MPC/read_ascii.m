% Program to read sample data from V40 logger.
% /Andreas Nilsson, 2010-03-11

function file_data=read_ascii(file_name)

fid = fopen(file_name);                      % open file
for i=1:7
    temp = fgetl(fid);                      % remove title rows
end
fscanf(fid,'%e',[13 inf]);                  % importing data
file_data = ans';
fclose(fid);                                % close file

disp('[ASCII data imported to matlab]')