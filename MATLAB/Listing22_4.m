count=0;
Z=zeros(size(1:20002));
DELAY=.015;

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
XKR=.1;
XKDC=(1.-XKR*XK3)/(XK1*XKR); %22_1에서 이거 변경

%actuactor transfer function
WACT=150;
ZACT=.7;

E=0;
ED=0;
DELD=0;
DEL=0;
THD=0;
DELC=0;
DELCP=0;
T=0;
H=.0001;
Z(1)=0;
I=1;
DINT=floor(DELAY/H); %ROUND(X) rounds the elements of X to the nearest integers.
S=0;
while ~(T > .99999)
    S=S+H;
    EOLD=E;
    EDOLD=ED;
    DELOLD=DEL;
    DELDOLD=DELD;
    STEP=1;
    FLAG=0;
    while STEP <=1
        if FLAG==1
            E=E+H*ED;
            ED=ED+H*EDD;
            DEL=DEL+H*DELD;
            DELD=DELD+H*DELDD;
            T=T+H;
            STEP=2;
        end
        DELCP=XKR*(XKDC*XNCG+THD);
        DELDD=WACT*WACT*(DELC-DEL-2.*ZACT*DELD/WACT);
        EDD=WAF*WAF*(DEL-E-2.*ZAF*ED/WAF);
        XNL=XK1*(E-EDD/WZ^2);
        THD=XK3*(E+TA*ED);
        FLAG=1;
    end
    FLAG=0;
    E=.5*(EOLD+E+H*ED);
    ED=.5*(EDOLD+ED+H*EDD);
    DEL=.5*(DELOLD+DEL+H*DELD);
    DELD=.5*(DELDOLD+DELD+H*DELDD);
    Z(I+1)=DELCP;
    if ((I+1) < DINT)
        DELC=Z(1);
    else
        DELC=Z(I+2-DINT); % I Have changed the code here from FORTRAN!
    end
    I=I+1;
    if S > .00099999
        S=0;
        count=count+1;
        ArrayT(count)=T;
        ArrayXNL(count)=XNL;
        ArrayXNCG(count)=XNCG;
    end
end
figure
plot(ArrayT,ArrayXNL,ArrayT,ArrayXNCG),grid
xlabel('Time (S)')
ylabel('Acceleration (G)')
title('Fig 22.17: Rate gyro flight control system ')
clc
output=[ArrayT', ArrayXNL', ArrayXNCG'];
save datfil.txt output /ascii
disp 'simulation finished'



