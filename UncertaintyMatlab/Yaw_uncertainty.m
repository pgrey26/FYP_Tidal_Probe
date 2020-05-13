% Yaw error

v = 1; %1 m/s

P = 0.5*997*v^2;

    
% P1, P3 constant at 1 m/s :
P1 = P; % pascals

P3 = P; % pascals

dP2 = -382: 1 :382; 

dP4 = dP2;

P2 = P1 + dP2(:);
P4 = P1 - dP4(:);

B_Px = [0 13.42 98.47];

 k = 0.0735;  % per degree, gradient from calibration plot
% C_dyn = 1.3; % around zero degrees

i = size(P4);
C_a = zeros(i);
a = zeros(i);
U_a = zeros(i);
P = zeros(i);
P(:) = P1;

i = length(C_a);

for m = 1 : 3 %98.48;
   
     B_P1 = B_Px(m);
     B_P2 = B_Px(m);
     B_P3 = B_Px(m);
     B_P4 = B_Px(m);

        for n = 1 : i
         C_a(n) = (P2(n) - P4(n))/ (0.25 * (P1 + P2(n) + P3 + P4(n)));

         a(n) = (1/k)*C_a(n);


           S_Px = 18.89; 
           S_P1 = S_Px; 
           S_P2 = S_Px;
           S_P3 = S_Px;
           S_P4 = S_Px;

         % Pitch C_b
        % C_b = (P1 - P3)/ (0.25 * (P1 + P2 + P3 + P4));
        % Dynamic C_dyn
        % C_dyn = (Po - P)/ (0.25 * (P1 + P2 + P3 + P4));

        % Partial Derivatives:

        da_dCa = 1 / k;

        db_dCb = 1 / k;

        da_dCb = 0;

        db_dCa = 0;


        % C_alpha
        dCa_dP1 = (4 * (P2(n) - P4(n))) / ((P1 + P2(n) + P3 + P4(n))^2);

        dCa_dP3 = dCa_dP1;

        dCa_dP2 = (4*P1 + 4*P3 + 8*P4(n)) / ((P1 + P2(n) + P3 + P4(n))^2);

        dCa_dP4 = (4*P1 + 8*P2(n) + 4*P3) / ((P1 + P2(n) + P3 + P4(n))^2);

        % C_beta
        dCb_dP1 = (4*P2(n) + 8*P3 + 4*P4(n)) / ((P1 + P2(n) + P3 + P4(n))^2);

        dCb_dP2 = (4 * (P1 - P3)) / ((P1 + P2(n) + P3 + P4(n))^2);

        dCb_dP4 = dCb_dP2;

        dCb_dP3 = (8*P1 + 4*P2(n) + 4*P4(n)) / ((P1 + P2(n) + P3 + P4(n))^2);

        % Error and Uncertainty Yaw (alpha)

        B_Ca = ((dCa_dP1)^2*B_P1^2 + (dCa_dP2)^2*B_P2^2 +(dCa_dP3)^2*B_P3^2 + ...
            (dCa_dP4)^2*B_P4^2)^0.5;

        S_Ca = ((dCa_dP1)^2*S_P1^2 + (dCa_dP2)^2*S_P2^2 +(dCa_dP3)^2*S_P3^2 + ...
            (dCa_dP4)^2*S_P4^2)^0.5;

        B_Cb = ((dCb_dP1)^2*B_P1^2 + (dCb_dP2)^2*B_P2^2 +(dCb_dP3)^2*B_P3^2 + ...
            (dCb_dP4)^2*B_P4^2)^0.5;

        S_Cb = ((dCb_dP1)^2*S_P1^2 + (dCb_dP2)^2*S_P2^2 +(dCb_dP3)^2*S_P3^2 + ...
            (dCb_dP4)^2*S_P4^2)^0.5;

        % Error and Uncertainty Yaw (alpha)
        B_a= ((da_dCa)^2 * B_Ca^2 + (da_dCb)^2 * B_Cb^2)^0.5;

        S_a = ((da_dCa)^2 * S_Ca^2 + (da_dCb)^2 * S_Cb^2)^0.5;

        U_a(n,m) = (B_a^2 + S_a^2)^0.5;  %Percent

        Ua(n,m) = (U_a(n,m) / abs(a(n)))*100;
        end
 
end      
    U_an = -U_a;

    
   

 figure(25);
 %linespec = ['linewidth', 1]; 
 plot(a, U_a,'LineWidth', 1.5); %,'Color', [0.29 0.40 0.43]);
 grid on
 legend('B_{Px} = 0 Pa','B_{Px} = 13.42 Pa', 'B_{Px} = 98.48 Pa'); 
 xlabel('\alpha / \beta (\circ)');
 ylabel ('\alpha / \beta Error (\pm\circ)');
 xlim([-20 20]);


 
 figure(31);
 hold on
 plot(a, Ua, 'LineWidth', 1.5);
 xlabel('\alpha / \beta (\pm \circ)');
 ylabel ('\alpha / \beta Error (\pm %)');
legend('B_{Px} = 0 Pa','B_{Px} = 13.42 Pa', 'B_{Px} = 98.48 Pa'); 
 ylim([0 100]);
 xlim([0 20]);
 grid on
 hold off
 
% Error and Uncertainty Pitch (beta)
% B_b(n,1) = ((db_dCa)^2 * B_Ca(n,1)^2 + (db_dCb)^2 * B_Cb(n,1)^2)^0.5;
% 
% S_b(n,1) = ((db_dCa)^2 * S_Ca(n,1)^2 + (db_dCb)^2 * S_Cb(n,1)^2)^0.5;
% 
% U_b(n,1) = (B_b(n,1)^2 + S_b(n,1)^2)^0.5; 
% 
