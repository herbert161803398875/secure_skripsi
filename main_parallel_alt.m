clear;
clc;
% SLX files: GFM_parallel.slx
%% Known Values
VLL_n = 380;         % Nominal RMS phase-to-phase Line Voltage (V)
fn    = 50;          % Nominal frequency (Hz)
fsw   = 1e4;         % Switching Frequency (Hz)
Sn    = 75e3;        % Nominal Apparent Power (VA)
Vdc   = 800;         % DC Voltage (V)
Ts    = 5e-5;        % Sampling Time (s)

%% Calculate Base Values
Sb  = Sn;
Vb  = VLL_n;
wb  = 2 * pi * fn;
wsw = 2 * pi * fsw;
Zb  = (Vb^2) / Sb;
Ib  = Sb / (Vb * sqrt(3));
Lb  = Zb / wb;
Cb  = 1 / (Zb * wb);

%% Calculate Lc
Imax  = sqrt(2) * Ib;
DImax = Imax / 10;
Lc    = Vdc / (6 * DImax * fsw);          
Lc_pu = Lc / Lb;                          
%% Calculate Cf
Cf_pu = 0.03;
Cf    = Cf_pu * Cb;                      
%% Calculate Lg Rg
Xc2Xg_ratio = 0.05;
Lgf         = 1 / (Xc2Xg_ratio * Cf * (wsw^2)); 
Lgf_pu      = Lgf/Lb;
Lg          = Lgf;
Lg_pu       = Lg / Lb;     

Rgf_pu      = 0;
Rgf         = 0;

XrXg        = 0.0;
Lgrid_pu    = 1;
Rgrid_pu    = Lgrid_pu*XrXg;
Rgrid       = Rgrid_pu *Zb;

%% Calculate Resonant Frequency
wres = sqrt((Lc + Lgf) / (Lc * Lgf * Cf));  % Resonant frequency (rad/s)
%% Calculate Rf Filter
Rf    = 1 / (3 * Cf * wres);              
Rf_pu = Rf / Zb;                          
%% Calculate Rc
Rc    = 1.0000e-05;
Rc_pu = Rc / Zb;

%% Calculate shunt
% shunt filter 
Zr  = Rf;
Zc  = 1i*(1/(2*pi*fn*Cf));
Zcf = Zr - Zc;
Ycf = 1/Zcf;                                   
Ypu = Ycf*Zb;                                  


Rs = Rf;
Cs = Cf;

Rs_pu = 0.1;
Cs_pu = 0.3;

Rs = Rs_pu*Zb;
Cs = Cs_pu*Cb;

Zrg  = Rs;
Zcg  = 1i*(1/(2*pi*fn*Cs));
Zcfg = Zrg - Zcg;
Ycfg = 1/Zcfg;                                  
Ypug = Ycfg*Zb; 


Zgrid_pu = Rgrid_pu+1i*Lgrid_pu;
scr      = 1/(abs(Zgrid_pu));
%% Calculate Choke filter parameters (pu)
X     = Lc_pu + Lgf_pu;                    
wLpu  = X;
%% Calculate Inner Current Control
w0    = 2 * pi * fn; 
L     = X / w0;
s     = tf('s');
plant = 1 / (Rc_pu + L * s);

f_bw  = 150;                              
kpi   = 2 * pi * f_bw * L;
kii   = Rc_pu / L * kpi;
%%  Calculate Inner Voltage Control (Voltage Cascade Loop)
wCpu  = Zb * wb * Cf;                      
wCpu = Cf_pu;
f_bwv = 5;

tVL   = tf(1/(Cf_pu*s)) ;                   

kpvd = 0.1; 
kivd = 50;

kpvq = 0.1;    
kivq = 50;

kpvd = 0.45; 
kivd = 50;

kpvq = 0.45;    
kivq = 50;   %untuk  double droop dan Line 0.5 pu (scr = 2)

%% Calculate Outer Loop Control Parameter     
Dp  = 0.02;
Dq  = 0.3;
H   = 0.3;
Kd  = 100; 
kiq = 0.01; 

Dp1 = Dp;
Dq1 = Dq;

Dp2 = Dp;
Dq2 = Dq;
%% load flow
% Power dissipation on shunt component : Pshunt=Gshunt×∣V∣^2 and Q-shunt = Bshunt×∣V∣^2

%************************ BUS DATA STARTS *********************************

% bus data format
% bus: number, voltage(pu), angle(degree), p_gen(pu), q_gen(pu), p_load(pu), q_load(pu),
% G-shunt/Shunt Conductance (pu), B shunt/Shunt Susceptance  (p.u); bus_type
%      bus_type - 1, swing bus
%               - 2, generator bus (PV bus)
%               - 3, load bus (PQ bus)
% ---------------------------------------------------------------------
bus = [...
    01 1.0       0.00   0.3    0.00    0.00   0.00    0.00       0.00       2  999  -999;  % Inverter Bus 1
    02 1.0       0.00   0.0    0.00    0.00   0.00    real(Ypu)  imag(Ypu)  3  999  -999;  % LCL Filter dan PCC inv 1
    03 1.0       0.00   0.0    0.00    0.60   0.10    real(Ypug) imag(Ypug) 3  999  -999;  % POC Bus
    04 1.0       0.00   0.2    0.00    0.00   0.00    0.00       0.00       2  999  -999;  % Inverter Bus 2
    05 1.0       0.00   0.0    0.00    0.00   0.00    real(Ypu)  imag(Ypu)  3  999  -999;  % LCL Filter dan PCC inv 2
    06 1.0       0.00   0.0    0.00    0.00   0.00    0.00       0.00       1  999  -999]; % inf Bus
%************************ BUS DATA ENDS *********************************

%************************ LINE DATA STARTS *******************************
line =[...
    01   2    Rc_pu     Lc_pu     0    1     0;
    02   3    Rgf_pu    Lgf_pu    0    1     0;  
    03   5    Rgf_pu    Lgf_pu    0    1     0;
    04   5    Rc_pu     Lc_pu     0    1     0;
    03   6    Rgrid_pu  Lgrid_pu  0    1     0];

deg_rad  = pi/180.0;                              % degree to radian,
rad_deg  = 180.0/pi;                              % radian to degree.
tol      = 1e-12;
iter_max = 200;
vmin     = 0.5; 
vmax     = 1.3; 
acc      = 1.0;
disply   ='y';
flag     = 2;
svolt    = bus(:,2); stheta = bus(:,3)*deg_rad;
bus_type = round(bus(:,10));

swing_index=find(bus_type==1);
    [bus_sol,line_sol,line_flow,Y1,y,tps,chrg] = ...
        loadflow(bus,line,tol,iter_max,acc,disply,flag);
%%%%%%%%%%%Load Flow End%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Network initialization 
V           = bus_sol(:,2);
theta       = bus_sol(:,3)*deg_rad;
first_theta = theta(1);
V0          = V.*(cos(theta)+ 1i*sin(theta));  % bus 1 converter, bus 2 lcl filter, bus 3 infinite bus
Vref        = V(3,1);

%% Control states intialization
% Voltage components
Vconv1  = V0(1,1);                                 
Vpcc1   = V0(2,1);
Vpoc    = V0(3,1);
Vconv2  = V0(4,1);                                 
Vpcc2   = V0(5,1);
Vinf    = V0(6,1);

% Current components
% with this If+Ic=Ig
Ic1    = (Vconv1-Vpcc1)/(Rc_pu+1i*Lc_pu);
Ic2    = (Vconv2-Vpcc2)/(Rc_pu+1i*Lc_pu);

If1    = (Vpcc1-Vpoc)/(Rgf_pu+1i*Lgf_pu);
If2    = (Vpcc2-Vpoc)/(Rgf_pu+1i*Lgf_pu);

Iinf   = (Vpoc-Vinf)/(Rgrid_pu+1i*Lgrid_pu);

Icf1   = -Vpcc1*Ypu;
Icf2   = -Vpcc2*Ypu;
Igf    = -Vpoc*Ypug;

Vcf1     = Vpcc1+ Rf_pu*Icf1;                         
Vcf2     = Vpcc2+ Rf_pu*Icf2;      
Vs       = Vpoc + Rs_pu*Igf;

% calculate constant impedance load 
YL       = diag((bus(:,6)-1i*bus(:,7))./V.^2);
Zload_pu = inv(YL(3,3));
I_load   = Vpoc/Zload_pu;

% Converting Associated Variables to Control RF
% Reference angle (pcc bus angle)
theta_ref = theta(3,1); 
% PCC voltage in the rotating reference frame :
Vpcc_c    = Vpoc * exp(-1i*theta_ref); 
% Conv voltage in the rotating reference frame:
Vconv_c1   = Vconv1 * exp(-1i*theta_ref);
Vconv_c2   = Vconv2 * exp(-1i*theta_ref);
% Grid current in the rotating reference frame:
Ifc1      = If1 * exp(-1i*theta_ref); 
Ifc2      = If2 * exp(-1i*theta_ref);            % Initial Value of current that goes to the grid 

% sesi 1
Pref1 = -line_flow(7,4);
Pref2 = -line_flow(3,4);
Qref1 = -line_flow(7,5);
Qref2 = -line_flow(3,5);

Pref1_2 = Pref1*1.3;
Pref2_2 = Pref2*1.3;
Qref1_2 = Qref1;
Qref2_2 = Qref2;

kfi    = 0;  
fac    = 0;


% outer loop states (GFL)
PsiP1    = real(Ifc1)-wCpu*imag(Vpcc_c)*fac+imag(Ifc1)*kfi;                          % (Old GFL)initial value of outer loop GFL (for P and Q)(not used)
PsiP2    = real(Ifc2)-wCpu*imag(Vpcc_c)*fac+imag(Ifc2)*kfi; 
PsiV1    = imag(Ifc1)+wCpu*real(Vpcc_c)*fac+real(Ifc1)*kfi;
PsiV2    = imag(Ifc2)+wCpu*real(Vpcc_c)*fac+real(Ifc2)*kfi;

% inner current loop states
PsiId1=real(Vconv_c1)-real(Vpcc_c)+X*imag(Ifc1);              % Initial value of Id in the current loop (inside the integrator)
PsiId2=real(Vconv_c2)-real(Vpcc_c)+X*imag(Ifc2);
PsiIq1=imag(Vconv_c1)-imag(Vpcc_c)-X*real(Ifc1);              % Initial value of Iq in the current loop (inside the integrator)
PsiIq2=imag(Vconv_c2)-imag(Vpcc_c)-X*real(Ifc2);  

%% 5) Tampilkan ringkasan hasil ke Command Window
totalsp1 = Pref1 + Pref2;
totalsq1 = Qref1 + Qref2;

totalsp2 = Pref1_2 + Pref2_2;
totalsq2 = Qref1_2 + Qref2_2;

fprintf('\n======= NILAI REFERENSI INVERTER (per-pu atau MW/Mvar) =======\n');
fprintf('                 Sesi-1\t\t\tSesi-2\n');
fprintf('Pref1   : %+8.4f\t\t%+8.4f\n', Pref1 ,Pref1_2);
fprintf('Pref2   : %+8.4f\t\t%+8.4f\n', Pref2 ,Pref2_2);
fprintf('Qref1   : %+8.4f\t\t%+8.4f\n', Qref1 ,Qref1_2);
fprintf('Qref2   : %+8.4f\t\t%+8.4f\n', Qref2 ,Qref2_2);
fprintf('-------------------------------------------------------------\n');
fprintf('TOTAL P : %+8.4f\t\t%+8.4f\n', totalsp1 , totalsp2);
fprintf('TOTAL Q : %+8.4f\t\t%+8.4f\n', totalsq1 , totalsq2);
fprintf('=============================================================\n');

% time_change = 5;
% [A,B,C,D,V,W,lambda,P] = ...
%     gfm_linearize('GFM_parallel',[],true,true,10,0.01);
