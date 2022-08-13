dt=6/1799;
K=35000;
KT=100000;
C=3000;
M=350;
m=50;
A=[0 1 0 0;
    -K/M -C/M K/M C/M;
    0 0 0 1;
    K/m C/m -(K+KT)/m -C/m];
B=[0;0;0;KT/m];
C_a=[-K/M -C/M K/M C/M];
C_p=[1 0 0 0];
D=0;
[num,den]=ss2tf(A,B,C_a,D);
acctf = tf(num,den);
[num,den]=ss2tf(A,B,C_p,D);
ptf = tf(num,den);
t=0:dt:6;
u1=0.05*(heaviside(t-1)-heaviside(t-2));
u2=0.05*heaviside(t-1);
pasv_p=lsim(ptf,u2,t);
pasv_a=lsim(acctf,u2,t);
%% passive part 5
j=1;
C_p=[1 0 0 0];
t=0:6/1799:6;
%KT =100000;
K=35000;
figure
%for K=[28000 35000 42000]
    for KT = [80000 100000 120000]
A=[0 1 0 0;
    -K/M -C/M K/M C/M;
    0 0 0 1;
    K/m C/m -(K+KT)/m -C/m];
B=[0;0;0;KT/m];
 C_a=[-K/M -C/M K/M C/M];
 [num,den]=ss2tf(A,B,C_a,D);
 acctf = tf(num,den);
[num,den]=ss2tf(A,B,C_p,D);
 ptf = tf(num,den);
z=lsim(ptf,u1,t);
a=lsim(acctf,u1,t);
subplot(2,3,j)
plot(t,z,t,u1,"--")
title("K_T = "+KT);xlabel("time (s)");xlim([0 4.5])
ylabel("posiztion (m)");ylim([-0.04 0.085])
subplot(2,3,j+3)
plot(t,a)
title("K_T = "+KT);xlabel("time (s)");xlim([0 4.5])
ylabel("accelaration (m/s^2)");ylim([-11 11])
j=j+1;
end

%% simulate active
A=[0 1 0 0 ;
   -K/M, -C/M, K/M, C/M;
   0, 0, 0, 1;
   K/m, C/m, -(K+KT)/m, -C/m];
B=[0 0;0 1/350;0 0;2000 -1/50];
C_a=[-K/M -C/M K/M C/M];
D=[0 1/350];
[num,den]=ss2tf(A,B,C_a,D,1);
acctfu = tf(num,den);
[num,den]=ss2tf(A,B,C_a,D,2);
acctfi = tf(num,den);
p=0.5;
I=1.02;
D=0.01;
ka= 4000;
De = tf([D 0],1);
PI = tf([p I],[1 0]);
PID = parallel(PI,De);
hyd=tf(ka,[0.05 1]);
FA=series(PID,hyd);
L1 = series(FA,acctfi);
G = feedback(1,L1);
sys = series(G,acctfu);
y=lsim(sys,u2,t); % get the accelaration  
z=cumsum(cumsum(y)*dt)*dt; % integrate to get the position 
subplot(1,2,1)
plot(t,z,t,pasv_p);legend("active","passive")%;title("response of active system")
ylim([-0.01 0.085]);ylabel("position (m)");xlabel("time (s)");
subplot(1,2,2)
plot(t,y,t,pasv_a);legend("active","passive")%;title("response of active system")
ylabel("accelaration (m/s^2)");xlabel("time (s)");
%% simulate additional 
j =1;
p=0.5;
I=1.02;
D=0.01;
%u3=[zeros(1,300),-0.1*(t(300:900)-1).*(t(300:900)-3),zeros(1,899)];
for k= 4000%[0 1000 4000 8000]
    De = tf([D 0],1);
    PI = tf([p I],[1 0]);
    PID = parallel(PI,De);
    hyd=tf(k,[0.05 1]);
    FA=series(PID,hyd);
    L1 = series(FA,acctfi);
    G = feedback(1,L1);
    sys = series(G,acctfu);
    y=lsim(sys,u1,t);
    z=cumsum(cumsum(y)*dt)*dt;
    %subplot(2,2,j)
    plot(t,y,t,u1,"--")
    title( " k = "+k )
    %ylim([-0.06 0.12])
    j=j+1;
end









