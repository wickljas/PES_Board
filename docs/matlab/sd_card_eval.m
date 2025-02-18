clc, clear variables
%% read the file, data contains delta as first entry

file_name = '001.bin';
data = read_sdcard_data(file_name)


%% evaluate the data

% // write data to the internal buffer of the sd card logger and send it to the sd card
% sd_logger.write(float(cntr)); // the logger only supports float, so we need to cast the counter to float
% sd_logger.send();
figure(1)
plot(data.values), grid on
xlabel('Time (sec)')
