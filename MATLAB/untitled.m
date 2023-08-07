fileID = fopen('sensordata.txt','r');
formatSpec = '%f %f';
sizeA= [2 Inf];
Aarray = fscanf(fileID,formatSpec,sizeA);
old_x=Aarray(1,1);
X= Aarray(1,:);
p=1;
for i=1:67
    A=1;R=0.003;H=2;Q=0.0004;
    
    input=Aarray(1,i);
    
    X(i)= A*old_x;
    
    p = A*p + Q;
    
    Kgain = p/(H*p+R);
    
    X(i) = Kgain*(input-X(i)) + X(i);
    
    p = (1-Kgain)*p;
    
    old_x=X(i);

end
plot(1:67,Aarray(1,:),1:67,Aarray(2,:),1:67,X),grid
xlabel('Time (Sec)')
ylabel('Acceleration (G)')