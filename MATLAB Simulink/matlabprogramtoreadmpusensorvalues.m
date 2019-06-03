%% setup 
hold all
a = arduino('COM4','Uno');
mpu=i2cdev(a,'0x68'); %mpu adress is normally 0x68
writeRegister(mpu, hex2dec('B6'), hex2dec('00'), 'int16'); %reset
data=zeros(10000,14,'int8'); %prelocating for the speed
j=1;
a1 = animatedline('Color',[1 0 0]); 
a2 = animatedline('Color',[0 1 0]);
a3 = animatedline('Color',[0 0 1]);
legend('Accel_x','Accel_y','Accel_z')

%% loop
while(true)
    x=1;
    for i=59:72 % 14 Data Registers for Accel,Temp,Gyro
        data(j,x)= readRegister(mpu, i, 'int8');
        x=x+1;
    end
    y=swapbytes(typecast(data(j,:), 'int16')) %if your system is big-endian remove the swapbytes function
    addpoints(a1,j,double(y(1)));
    addpoints(a2,j,double(y(2)));
    addpoints(a3,j,double(y(3)));
    j=j+1;
    drawnow limitrate
end