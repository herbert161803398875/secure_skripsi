clear;
clc;
%% Known Values
VLL_n = 380;      % Nominal RMS phase-to-phase Line Voltage (V)
fn    = 50;          % Nominal frequency (Hz)
fsw   = 1e4;        % Switching Frequency (Hz)
Sn    = 75e3;        % Nominal Apparent Power (VA)
Vdc   = 800;        % DC Voltage (V)
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
Dp_pu = Sb/wb;
%% Calculate Lc
Imax  = sqrt(2) * Ib;
DImax = Imax*0.1;
Lc    = Vdc / (6 * DImax * fsw);          % Inductance (H) 
Lc_pu = Lc / Lb;                          % Per-unit inductance

%% Calculate Cf
Cf_pu = 0.03;
Cf    = Cf_pu * Cb;                       % Capacitance (F)
%% Calculate Lg
Xc2Xg_ratio = 0.05;
Lgs         = 1 / (Xc2Xg_ratio * Cf * (wsw^2)); % Inductance (H)
Lg_line     = 0.0040; %0.0049-0.001 ;
Lg          = Lgs + Lg_line;
Lg_pu       = Lg/Lb;                            % Per-unit inductance
Lg_pu       = Lg_pu; 
ka = 0.1;
term1 = Lc / ((0.5 * wsw)^2 * Lc * Cf - 1);
term2 = (1 + 1/ka) / (Cf * wsw^2);
% Tentukan nilai Lg minimal
Lg_min = max(term1, term2);
%% Calculate Resonant Frequency
wres = sqrt((Lc + Lgs) / (Lc * Lgs * Cf));  % Resonant frequency (rad/s)
%% Calculate Rf
Rf    = 1 / (3 * Cf * wres);                  % Resistance (ohms)
Rf_pu = Rf / Zb;                              % Per-unit resistance
%% Calculate Rc
Rc    = 1.0000e-05;
Rc_pu = Rc / Zb;
XrXg  = 0;
Rg_pu = Lg_pu*XrXg;
Rg    = Rg_pu *Zb;
scr   = 1/abs(Rg_pu+1i*Lg_pu);
r_x   = Rg_pu/Lg_pu;
%% Calculate Choke filter parameters (pu)
X     = Lc_pu + Lgs/Lb;                        % Choke filter parameters (pu)
wLpu  = X;
%% Calculate Inner Current Control
w0    = 2 * pi * fn; 
L     = X / w0;
s     = tf('s');
plant = 1 / (Rc_pu + L * s);

f_bw  = 150;                                  % Desired bandwidth (Hz)
kpi   = 2 * pi * f_bw * L;
kii   = Rc_pu / L * kpi;

%%  Calculate Inner Voltage Control (Voltage Cascade Loop)
wCpu  = Zb * wb * Cf;                      
f_bwv = 5;
tVL   = tf(1/(Cf_pu*s)) ;                  % voltage cascade loop transfer function 

%% KPV KIV 
kpvd = 0.1; %untuk scr 2.89 lg 29 work untuk droop juga
kivd = 100;

kpvq =0.1; %untuk scr 2.89 lg 29 work untuk droop juga
kivq =100;

%% Outer Loop Control Parameter 
Dp  = 0.02;
Dq  = 0.3;
Kd  = 50;
H   = 1.2 ; %s
kiq = 0.1; 
J   = 2*H*Sb/(wb)^2;
mu  = 0.005;
%% load flow
Zr  = Rf;
Zc  = 1i*(1/(2*pi*fn*Cf));
Zcf = Zr - Zc;
Ycf = 1/Zcf;                                   % admitance RC filter
Ypu = Ycf*Zb;                                  % (Gshunt + jBshunt p.u.)
Lg_pu = 0.5;
XrXg = 0;
Rg_pu = XrXg*Lg_pu;

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
    01 1.0       0.00   1.0    0.00    0.00   0.00    0.00      0.00       2  999  -999;  % Type 2 (PV)(Inverter Bus)
    02 1.0       0.00   0.0    0.00    0.40   0.20    real(Ypu) imag(Ypu)  3  999  -999;  % LCL Filter Bus
    03 1.0       0.00   0.0    0.00    0.00   0.00    0.00      0.00       1  999  -999]; % Slack(SMIB Grid) Bus
%************************ BUS DATA ENDS *********************************

%************************ LINE DATA STARTS *******************************
line =[...
    01   2    Rc_pu     Lc_pu     0    1     0;
    02   3    Rg_pu     Lg_pu     0    1     0]; % Saluran 2
%************************ LINE DATA ENDS *******************************

deg_rad  = pi/180.0;                              % degree to radian,
rad_deg  = 180.0/pi;                              % radian to degree.
tol      = 1e-12;
iter_max = 50;
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
Pref        = -line_flow(3,4);
Qref        = -line_flow(3,5);
Vref        = V(3,1);
%% Control states intialization
% Voltage components
Vconv = V0(1,1);                                % Converter control block initial value 
Vlcl  = V0(2,1);
Vinf  = V0(3,1);
Vpcc0 = Vlcl;

% Current components
% with this If+Ic=Ig
Ic         = (Vconv-Vlcl)/(Rc_pu+1i*Lc_pu);          % Initial value of current that pass the LcRc
Z_pu       = Rg_pu + 1i*Lg_pu;
Ig         = (Vlcl - Vinf) / Z_pu;
Icf        = -Vlcl*Ypu;
Vcf0       = Vlcl+Rf_pu*Icf;                         % Initial value of Vpcc

% Converting Associated Variables to Control RF
% Reference angle (infinite bus angle)
theta_ref = theta(2,1); 
% PCC voltage in the rotating reference frame :
Vpcc_c    = Vlcl * exp(-1i*theta_ref); 
% Conv voltage in the rotating reference frame:
Vconv_c   = Vconv * exp(-1i*theta_ref);
% Grid current in the rotating reference frame:
Ipcc_c = Ic * exp(-1i*theta_ref);           % Initial Value of current that goes to the grid      
kfi    = 0;  
kffi   = 0;
fac    = 0;

YL       = diag((bus(:,6)-1i*bus(:,7))./V.^2)    ;%Constant Impedence Load model
Zload_pu = inv(YL(2,2));
I_load   = Vpcc0/Zload_pu;
I_loadc  = I_load * exp(-1i*theta_ref); 

% UNTUK MAIN SCR MAKA 1. LIHAT PERHITUNGAN SCR, 2. LIHAT X/R ATAU R/X DI IEC 141-1993

% outer loop states (GFL)
PsiP0    = real(Ipcc_c)-wCpu*imag(Vpcc_c)*fac+imag(Ipcc_c)*kfi; % (Old GFL)initial value of outer loop GFL (for P and Q)(not used)
PsiV0    = imag(Ipcc_c)+wCpu*real(Vpcc_c)*fac+real(Ipcc_c)*kfi;

% inner current loop states
PsiId0=real(Vconv_c)-real(Vpcc_c)+X*imag(Ipcc_c);              % Initial value of Id in the current loop (inside the integrator)
PsiIq0=imag(Vconv_c)-imag(Vpcc_c)-X*real(Ipcc_c);              % Initial value of Iq in the current loop (inside the integrator)
  
% inner voltage loop states 
psiVd0 = real(Ipcc_c)+imag(Vpcc_c)*wCpu;                       % Initial value of Vd in the voltage loop (inside the integrator)
psiVq0 = imag(Ipcc_c)-real(Vpcc_c)*wCpu;                       % Initial value of Vq in the voltage loop (inside the integrator)


% initial controller delay values for converters
% Just used when we use abc controller (in this case we dont use this at all)
wt0=0;
Matrix= [sin(wt0) cos(wt0) 1;
        sin(wt0-2*pi/3) cos(wt0-2*pi/3) 1;
        sin(wt0+2*pi/3) cos(wt0+2*pi/3) 1];      % dq to abc frame matrix (inverse park transform)
uabc0 = Vb*sqrt(2/3)*Matrix*[real(Vconv);imag(Vconv);0]/Vdc/2;


% % Linearise 'test_ss_infload.slx', hitung P, tampilkan tabel, tanpa plot
% [A,B,C,D,V,W,lambda,P] = ...
%     gfm_linearize('test_ss_infload1',[],true,true,10,0.01);

