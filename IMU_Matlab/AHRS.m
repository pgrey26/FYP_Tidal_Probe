% % AHRS
for i=1:4
    if i==1
        file='Logger26.mat';
    elseif i==2
        file='Logger27.mat'; 
    elseif i==3
        file='Logger28.mat'; 
    elseif i==4
        file='Logger29.mat';   
    end
ld = load(file);

acc = ld.sensorData.Acceleration;
gyro = ld.sensorData.AngularVelocity;
mag = ld.sensorData.MagneticField;

viewer = HelperOrientationViewer;


%Input Parameters
Fs = ld.Fs;
GyroscopeNoise = 6.0462e-05; % GyroscopeNoise (variance value) in units of rad/s
AccelerometerNoise = 0.0061; % AccelerometerNoise(variance value)in units of m/s^2
Mn = 68.68; %1.761;%magnetometerNoise
GDN = 3e-13;%GyroscopeDriftNoise
%LinearAccelerationNoise
LinearAccelerationDecayFactor = 0.3;


OrientationFormat = 'quaternion'; %'Rotation matrix' or 'quaternion'
ifilt = imufilter('SampleRate',Fs, 'GyroscopeNoise',GyroscopeNoise,...
    'AccelerometerNoise',AccelerometerNoise);



q = ifilt(acc, gyro);
Fs = ld.Fs;
time = (0:1:size(acc,1)-1)/Fs;

figure()
plot(time,eulerd(q,'XYZ','frame'))
text = sprintf('Orientation Estimate, filename: %s', file);
title(text)
legend('x-axis', 'y-axis', 'Z-axis')
ylabel('Rotation (degrees)')

numSamples = size(acc,1);
time = (0:(numSamples-1))'/Fs;

figure()
subplot(3,1,1)
plot(time,acc)
text = sprintf('Accelerometer Reading, filename: %s', file);
title(text)
ylabel('Acceleration (m/s^2)')

subplot(3,1,2)
plot(time,mag)
title('Magnetometer Reading')
ylabel('Magnetic Field (\muT)')

subplot(3,1,3)
plot(time,gyro)
title('Gyroscope Reading')
ylabel('Angular Velocity (rad/s)')
xlabel('Time (s)')

end