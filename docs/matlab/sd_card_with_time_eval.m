clc, clear variables
%% read the file, data contains delta as first entry

file_name = '002.bin';
data = read_sdcard_data_with_time(file_name);


%% evaluate the data

% average sampling time
Ts = mean(diff(data.time));

figure(1)
plot(data.time(1:end-1), diff(data.time * 1e6)), grid on
title( sprintf(['Mean %0.0f mus, ', ...
                'Std. %0.0f mus, ', ...
                'Med. dT = %0.0f mus'], ...
                mean(diff(data.time * 1e6)), ...
                std(diff(data.time * 1e6)), ...
                median(diff(data.time * 1e6))) )
xlabel('Time (sec)'), ylabel('dTime (mus)')
xlim([0 data.time(end-1)])
ylim([0 1.2*max(diff(data.time * 1e6))])

% // write data to the internal buffer of the sd card logger and send it to the sd card
% sd_logger.write(dtime_us);
% sd_logger.write(ir_distance_mV);
% sd_logger.write(ir_distance_cm);
% sd_logger.write(ir_distance_avg);
% sd_logger.send();
figure(2)
ax(1) = subplot(121);
plot(data.time, data.values(:,1)), grid on
xlabel('Time (sec)')
legend('Distance in mV')
ax(2) = subplot(122);
plot(data.time, [data.values(:,2), data.values(:,3)]), grid on
xlabel('Time (sec)')
legend('Distance (cm)', 'Distance Averaged (cm)')
linkaxes(ax, 'x'), clear ax
xlim([0 data.time(end)])
