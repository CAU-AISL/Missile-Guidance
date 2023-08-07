ount=0;

%missile spec
VM=3000.; %미사일 속도
A=1000.;% 소리 속도
XMACH=VM/A; %마하 값
B=sqrt(XMACH^2-1);% normal force 방정식 매개변수
ALT=0.; % 고도
DIAM=1.; %동체 지름
FR=3.; %Radome 길이
XL=20.; % 로켓 길이
XHL=19.5; % Hinge까지 길이
XCG=10.; % 무게중심까지 길이 (가정)
XACC=XCG;
CTW=0.; % Wing tip length
CRW=6.; % Wing root length
HW=2.; % Wing height
CTT=0.; % Tail tip length
CRT=2.; % Tail root length
HT=2.; % Tail height
XN=4.;
WGT=1000.;% weight missile
if ALT<=30000.
    RHO=.002378*exp(-ALT/30000.);% 밀도
else
    RHO=.0034*exp(-ALT/22000.); % 밀도
end
SWING=.5*HW*(CTW+CRW);% wing area
STAIL=.5*HT*(CTT+CRT);% tail area
SREF=3.1416*DIAM*DIAM/4.;%Reference area
XLP=FR*DIAM;%L'
SPLAN=(XL-XLP)*DIAM+1.33*XLP*DIAM/2.;%Planform Area
XCPN=2*XLP/3;% nose의 압력중심
AN=.67*XLP*DIAM;% nose Area(?)
AB=(XL-XLP)*DIAM;% Body Area
XCPB=(.67*AN*XLP+AB*(XLP+.5*(XL-XLP)))/(AN+AB);% body의 압력중심
XCPW=XLP+XN+.7*CRW-.2*CTW;% wing의 압력중심
XIYY=WGT*(3*((DIAM/2)^2)+XL*XL)/(12*32.2);% 미사일 관성 모멘트
Q=.5*RHO*VM*VM;%동력압력
%missile spec

TMP1=(XCG-XCPW)/DIAM;
TMP2=(XCG-XHL)/DIAM;
TMP3=(XCG-XCPB)/DIAM;
TMP4=(XCG-XCPN)/DIAM;

TMP1=(XCG-XCPW)/DIAM;
TMP2=(XCG-XHL)/DIAM;
TMP3=(XCG-XCPB)/DIAM;
TMP4=(XCG-XCPN)/DIAM;

XNCG=10.;%Command Acceleration
P1=WGT*XNCG/(Q*SREF); %C_NTRIM
 
% Y series
Y1=2.+8*SWING/(B*SREF)+8*STAIL/(B*SREF);
Y2=1.5*SPLAN/SREF;
Y3=8*STAIL/(B*SREF);
Y4=2*TMP4+8*SWING*TMP1/(B*SREF)+8*STAIL*TMP2/(B*SREF);
Y5=1.5*SPLAN*TMP3/SREF;
Y6=8*STAIL*TMP2/(B*SREF);
% Y series

% P series
P2=Y2-Y3*Y5/Y6;
P3=Y1-Y3*Y4/Y6;
% P series

%trim factor
ALFTR=(-P3+sqrt(P3*P3+4.*P2*P1))/(2.*P2); %angle of attack trim
DELTR=-Y4*ALFTR/Y6-Y5*ALFTR*ALFTR/Y6; %deflection trim

%airframe transfer function
CNA=2+1.5*SPLAN*ALFTR/SREF+8*SWING/(B*SREF)+8*STAIL/(B*SREF);
CND=8*STAIL/(B*SREF);
CMAP=2*TMP4+1.5*SPLAN*ALFTR*TMP3/SREF+8*SWING*TMP1/(B*SREF);
CMA=CMAP+8*STAIL*TMP2/(B*SREF);
CMD=8*STAIL*TMP2/(B*SREF);
XMA=Q*SREF*DIAM*CMA/XIYY;
XMD=Q*SREF*DIAM*CMD/XIYY;
ZA=-32.2*Q*SREF*CNA/(WGT*VM);
ZD=-32.2*Q*SREF*CND/(WGT*VM);
WZ=sqrt((XMA*ZD-ZA*XMD)/ZD);
WAF=sqrt(-XMA);
ZAF=.5*WAF*ZA/XMA;
XK1=-VM*(XMA*ZD-XMD*ZA)/(1845*XMA);
XK2=XK1;
TA=XMD/(XMA*ZD-XMD*ZA);
XK3=1845*XK1/VM;
%airframe transfer function

%auto-pilot function
WCR=50.; %open loop crossover frequency
ZETA=.7; % damping 
TAU=.3; % time constant
W=(TAU*WCR*(1+2.*ZAF*WAF/WCR)-1)/(2*ZETA*TAU);
W0=W/sqrt(TAU*WCR);
Z0=.5*W0*(2*ZETA/W+TAU-WAF^2/(W0*W0*WCR));
XKC=(-W0^2/WZ^2-1.+2.*Z0*W0*TA)/(1.-2.*Z0*W0*TA+W0*W0*TA*TA);
XKA=XK3/(XK1*XKC); % autopilot gain
WI=XKC*TA*W0*W0/(1+XKC+W0^2/WZ^2);% autopilot gain
XK0=-W*W/(TAU*WAF*WAF);
XK=XK0/(XK1*(1+XKC));
XKR=XK/(XKA*WI);% autopilot gain
XKDC=1.+1845./(XKA*VM);  % autopilot gain
%auto-pilot function

%actuactor transfer function
WACT=150;
ZACT=.7;

for I=2:160
    W=10^(.025*I-1);
    XMAGTOP=-XK0*sqrt((1.-(W/W0)^2)^2+(2.*Z0*W/W0)^2);
    XMAGBOT=W*sqrt((1.-(W/WAF)^2)^2+(2.*ZAF*W/WAF)^2);
    XMAG=XMAGTOP/XMAGBOT;
    XMAGACT=1./sqrt((1.-W*W/(WACT*WACT))^2+(2.*ZACT*W/WACT)^2);
    PHASETOP=atan2(2.*Z0*W/W0,1.-(W/W0)^2);
    PHASEBOT=atan2(2.*ZAF*W/WAF,1.-(W/WAF)^2);
    PHASEACT=atan2(2.*ZACT*W/WACT,1.-W*W/(WACT*WACT));
    GAIN=20.*log10(XMAG*XMAGACT);
    PHASE=-90.+57.3*(PHASETOP-PHASEBOT-PHASEACT);
    count=count+1;
    ArrayW(count)=W;
    ArrayGAIN(count)=GAIN;
    ArrayPHASE(count)=PHASE;
end
figure
semilogx(ArrayW,ArrayGAIN),grid
xlabel('Frequency (Rad/Sec)')
ylabel('Gain (Db)')
axis([.1 1000 -60 40])
figure
semilogx(ArrayW,ArrayPHASE),grid
xlabel('Frequency (Rad/Sec)')
ylabel('Phase (Deg)')
axis([.1 1000 -400 100])
clc
output=[ArrayW',ArrayGAIN',ArrayPHASE'];
save datfil.txt output /ascii
disp ’simulation finished’