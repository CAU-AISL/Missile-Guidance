% Gaussian distribution 로 얻은 100개의 변수

count=0;
N=100;
for I=1:N
    SUM=0;
    for J=1:12
        RAND=rand(1); % 0부터 1사이의 변수
        SUM=SUM+RAND; % 12개를 다 더하기
    end
    X=SUM-6; % 합에서 6 빼기
    count=count+1;
    ArrayI(count)=I;
    ArrayX(count)=X;
end

figure
plot(ArrayI,ArrayX),grid
title('One hundred random numbers with Gaussian distribution')
xlabel('Number')
ylabel('Value')
clc
output=[ArrayI,ArrayX];
save datfil output -ascii
disp 'simulation finished'