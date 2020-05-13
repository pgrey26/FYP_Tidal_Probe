%AHRS Visualisation with logged IMU data
for i=2
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
GyroscopeNoise = 3.0462e-06; % GyroscopeNoise (variance value) in units of rad/s
AccelerometerNoise = 0.0061; % AccelerometerNoise(variance value)in units of m/s^2
Mn = 3.35; %1.761;%magnetometerNoise
GDN = 0.00003;%0.0005;%GyroscopeDriftNoise
%LinearAccelerationNoise
LinearAccelerationDecayFactor = 0.1;
%MagneticDisturbanceNoise
MDD = 0.5; %MagneticDisturbanceDecayFactor
MDN = 0.5;
expMFS = 60; %ExpectedMagneticFieldStrength
OrientationFormat = 'quaternion'; %'Rotation matrix' or 'quaternion

%Create AHRS Filter Object


ifilt = ahrsfilter('SampleRate',Fs, 'GyroscopeNoise',GyroscopeNoise,...
    'AccelerometerNoise',AccelerometerNoise);
   % 'MagnetometerNoise', Mn, 'MagneticDisturbanceNoise', MDN, 'MagneticDisturbanceDecayFactor', MDD);
 
%qahrs = ifilt(acc, gyro, mag);
    for ii=1:size(acc(:,1))
        qahrs = ifilt(acc(ii,:), gyro(ii,:), mag(ii,:));
        viewer(qahrs);
        pause(1/Fs);
    end


end