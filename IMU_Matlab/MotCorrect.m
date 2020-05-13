file = 'Logger34.mat';
ld = load(file);

acc = ld.sensorData.Acceleration;
gyro = ld.sensorData.AngularVelocity;
mag = ld.sensorData.MagneticField;

viewer = HelperOrientationViewer;


%Input Parameters
Fs = ld.Fs;
GyroscopeNoise = 3.0462e-06; % GyroscopeNoise (variance value) in units of rad/s
AccelerometerNoise = 0.0061; % AccelerometerNoise(variance value)in units of m/s^2
%magnetometerNoise
%GyroscopeDriftNoise
%LinearAccelerationNoise
%LinearAccelerationDecayFactor
%MagneticDisturbanceNoise
%MagneticDisturbanceDecayFactor
%ExpectedMagneticFieldStrength
OrientationFormat = 'Rotation matrix'; %'Rotation matrix' or 'quaternion'

%Create AHRS Filter Object
ifilt = ahrsfilter('SampleRate',Fs, 'GyroscopeNoise',GyroscopeNoise,...
    'AccelerometerNoise',AccelerometerNoise, 'OrientationFormat', OrientationFormat);
R = zeros(3,3,size(acc,1));
Rt = zeros(3,3,size(acc,1));
UhE = zeros(size(acc));
Uh = zeros(size(acc));

  acc(:,1) = acc(:,1) - mean(acc(1:15,1));
  acc(:,2) = acc(:,2) -  mean(acc(1:10,2));
  acc(:,3) = acc(:,3) - mean(acc(1:10,3));

ell= [0 0 0];
time = (0:1:size(acc,1)-1)/Fs;
dt = 1/Fs;
fpass = 0.333;

%  for ii=1:size(acc,1)
%      R(:,:,ii) = ifilt(acc(ii,:), gyro(ii,:), mag(ii,:));
%     Rt(:,:,ii) = transpose(R(:,:,ii));
%     accE(ii,:) = acc(ii,:)*Rt(:,:,ii);
%  end
 

 acchp = highpass(acc, fpass, Fs);
 Ua = cumtrapz(dt,acchp);

 
 Uh = Ua;

%  for ii=1:size(acc,1)
%  R(:,:,ii) = ifilt(acc(ii,:), gyro(ii,:), mag(ii,:));
%      Rt(:,:,ii) = transpose(R(:,:,ii));
%    
%     Uh(ii,:) = (cross(gyro(ii,:), ell)) + intacc(ii,:) ;
%     UhE(ii,:) = Uh(ii,:) * Rt(:,:,ii);
%  end
 figure()
 plot(time, Uh);
hold on
 ylabel('Velocity (m/s)');
 xlabel('time (s)');
 legend('U_{hx}', 'U_{hy}', 'U_{hz}');
 txt = sprintf('filname: %s;  fpass = %.3f', file, fpass);
 title(txt)

 hold off
 
 
%   figure()
%  plot(time, UhE);
%  ylabel('Velocity (m/s)');
%  xlabel('time (s)');
%  legend('U_{hx}', 'U_{hy}', 'U_{hz}');
%  earth frame
% 
% figure()
% plot(time,acc(:,1:2));%, time, acchp(:,1:2))
% legend('ax', 'ay');%, 'axhp', 'ayhp');

 