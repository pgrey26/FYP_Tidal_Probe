%Velocity Error

v = linspace(0,3,500); %1 m/s

i = length(v);
U_u = zeros(i,1);

B_Px = [0 13.42 98.47];

for m = 1:3  
     for n = 1:i
         
         rho = 997; %density kg/m3
         
         Px = 0.5*1030*v(n)^2;
         
         C_dyn = 1.3;
        
         S_Px =  18.89; 
         
         U_Px = (B_Px(m)^2 + S_Px^2)^0.5;
         
         %Dynamic Pressure
         dPdyn_Cdyn = Px;
         
         dPdyn_dPx = 0.25*C_dyn;
         
         %Zero for small angles:
         dCdyn_dCa = 0;
         dCdyn_dCb = 0;
         
         % B_Cdyn = ((dCdyn_dCa)^2*B_Ca^2 + (dCdyn_dCb)^2*B_Cb^2)^0.5;
         % S_Cdyn = ((dCdyn_dCa)^2*S_Ca^2 + (dCdyn_dCb)^2*S_Cb^2)^0.5;
         B_Cdyn = 0;
         S_Cdyn = 0;
         
         
         B_dyn =((dPdyn_Cdyn)^2 * B_Cdyn^2 +(dPdyn_dPx)^2*B_Px(m)^2 + ...
             (dPdyn_dPx)^2*B_Px(m)^2 +(dPdyn_dPx)^2*B_Px(m)^2 +(dPdyn_dPx)^2*B_Px(m)^2)^0.5;
         
         S_dyn =((dPdyn_Cdyn)^2 * S_Cdyn^2 +(dPdyn_dPx)^2*S_Px^2 + ...
             (dPdyn_dPx)^2*S_Px^2 +(dPdyn_dPx)^2*S_Px^2 +(dPdyn_dPx)^2*S_Px^2)^0.5;
         
         U_dyn = (B_dyn^2 + S_dyn^2)^0.5;
         
         
         du_dPdyn = (2*rho*Px)^(-0.5);
         
         B_u = ((du_dPdyn)^2 * B_dyn^2)^0.5;
         S_u = ((du_dPdyn)^2 * S_dyn^2)^0.5;
         
         U_u(n,m) = (B_u^2 + S_u^2)^0.5;
         
         Uu(n,m) = (U_u(n,m) / v(n))*100;
     end
end



figure(21)
plot(v, U_u,  'LineWidth', 1.5); %'Color', [0.29 0.40 0.43]
hold on
xlabel('Velocity (m/s)');
ylabel('Velocity error (\pm m s^{-1}'); 
xlim([0 1.5]);
ylim([0 1.5]);
legend('B_{Px} = 0 Pa', 'B_{Px} = 13.42 Pa', 'B_{Px} = 98.48 Pa'); 
grid on
hold off

figure(22)
plot(v, Uu,'LineWidth', 1.5); % , 'Color', [0.29 0.40 0.43]); 
hold on
xlabel('Velocity (m s^{-1})');
ylabel('Velocity error (\pm %)'); 
xlim([0 1.5]);
ylim([0 100]);
legend('B_{Px} = 0 Pa','B_{Px} = 13.42 Pa', 'B_{Px} = 98.48 Pa'); 
grid on