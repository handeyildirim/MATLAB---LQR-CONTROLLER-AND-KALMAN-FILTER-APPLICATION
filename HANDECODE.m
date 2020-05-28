% _Hande Yildirim, 141201047, Ele515, Final Proje_

% BOLUM1

% Oncelikle tasarlanacak sistem matrisleri ve constant degerler yazildi

M1 = 20; %M1 agirlik
M2 = 40; %M2 agirlik
Ks = 4; %yay sabiti

% sistem matrisleri
A = [ 0 1 0 0 ; 0 0 1 0 ; 0 0 0 1 ; 0 0 (-Ks * (M1+M2)/M1 * M2) 0];
B = [0 0 ; 0 0 ; 0 0 ; 1/M1 Ks/(M1*M2)];
C = [1 0 0 0];
D = [0];

% state space sistemi olusturuldu
sistem = ss(A,B,C,D)

% Matlabda Sistem Simulasyonu
figure(1)
subplot(2,1,1)
step(sistem) %girdi olarak verilen birim basamaga gore sistem ciktisinin cizilmesi
title('MATLAB''da Sistem Cevabı')

% Simulinkte Sistem Simulasyonu
subplot(2,1,2)
model = sim('projesim1.slx'); %simulinkten dosyayı cagir
t = model.tout; %simulink zamanini belirle
sistem_out = model.sistem_out;
plot(sistem_out) %simulinke gore sistem ciktisi
title('Simulink''te Sistem Cevabı')


% BOLUM 2

% System Linearized Output With Kalman Filter
% System Output From Simulink*

model = sim('KALMAN1.slx');%model simulinkten cekilir
t = model.tout;%zaman cekilir

% d1 icin Kalman Filtresi uygulanmış toWorkspaceler Scripte cekilir
kal1 = model.kal1;
kal2 = model.kal2;
kal3 = model.kal3;

sist_out1 = model.out1;
sist_out2 = model.out2;
sist_out3 = model.out3;

sistem_out = model.sistem_out;

% Yeni pencerede grafik cizimi icin figure olusturulur ve sonrasinda sistem cizilir
figure(2)

subplot(2,2,1)
plot(kal1)%d1 durum Q=1 R=1 icin cizdir
hold on
plot(sist_out1')%kalman filtresli sistem cikisi Q=1 R=1 icin cizdir
hold off
title('d1 durumu icin: Q = 1 R = 1')

subplot(2,2,2)
plot(kal2)%d1 durum Q=1 R=1000 icin cizdir
hold on
plot(sist_out2)%kalman filtresli sistem cikisi Q=1 R=1000 icin cizdir
hold off
title('d1 durumu icin: Q = 1 R = 1000')

subplot(2,2,3)
plot(kal3)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold on
plot(sist_out3)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold off
title('d1 durumu icin: Q = 1000 R = 1')

subplot(2,2,4)
hold on
plot(sistem_out)%kalman filtresi olmayan sistem cikisi cizdir
title('Tasarlanan sistem')
hold off

% x1 icin Kalman Filtresi uygulanmış toWorkspaceler Scripte cekilir

%kalman filtresi x1 durum ciktilari
kal1x = model.kal1x;
kal2x = model.kal2x;
kal3x = model.kal3x;

sist_out1 = model.out1;
sist_out2 = model.out2;
sist_out3 = model.out3;

sistem_out = model.sistem_out; %filtresiz sistem ciktisi

% Sonrasinda kalman filtreli ciktilar cizdirilir
figure(3)

subplot(2,2,1)
plot(kal1x)%x1 durum Q=1 R=1 icin cizdir
hold on
plot(sist_out1')%kalman filtresli sistem cikisi Q=1 R=1 icin cizdir
hold off
title('x1 durumu icin: Q = 1 R = 1')

subplot(2,2,2)
plot(kal2x)%x1 durum Q=1 R=1000 icin cizdir
hold on
plot(sist_out2)%kalman filtresli sistem cikisi Q=1 R=1000 icin cizdir
hold off
title('x1 durumu icin: Q = 1 R = 1000')

subplot(2,2,3)
plot(kal3x)%x1 Q=1000 R=1 icin cizdir
hold on
plot(sist_out3)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold off
title('x1 durumu icin: Q = 1000 R = 1')

subplot(2,2,4)
hold on
plot(sistem_out)%kalman filtresi olmayan sistem cikisi cizdir
title('Tasarlanan sistem')
hold off

% a1 icin Kalman Filtresi uygulanmış toWorkspaceler Scripte cekilir
kal1a = model.kal1a;
kal2a = model.kal2a;
kal3a = model.kal3a;

sist_out1 = model.out1;
sist_out2 = model.out2;
sist_out3 = model.out3;

sistem_out = model.sistem_out;

figure(4)

subplot(2,2,1)
plot(kal1a)%a1 durum Q=1 R=1 icin cizdir
hold on
plot(sist_out1')%kalman filtresli sistem cikisi Q=1 R=1 icin cizdir
hold off
title('a1 durumu icin: Q = 1 R = 1')

subplot(2,2,2)
plot(kal2a)%a1 durum Q=1 R=1000 icin cizdir
hold on
plot(sist_out2)%kalman filtresli sistem cikisi Q=1 R=1000 icin cizdir
hold off
title('a1 durumu icin: Q = 1 R = 1000')

subplot(2,2,3)
plot(kal3a)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold on
plot(sist_out3)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold off
title('a1 durumu icin: Q = 1000 R = 1')

subplot(2,2,4)
hold on
plot(sistem_out)%kalman filtresi olmayan sistem cikisi cizdir
title('Tasarlanan sistem')
hold off

% v1 icin Kalman Filtresi uygulanmış toWorkspaceler Scripte cekilir
kal1v = model.kal1v;
kal2v = model.kal2v;
kal3v = model.kal3v;

sist_out1 = model.out1;
sist_out2 = model.out2;
sist_out3 = model.out3;

sistem_out = model.sistem_out;

figure(5)

subplot(2,2,1)
plot(kal1v)%v1 durum Q=1 R=1 icin cizdir
hold on
plot(sist_out1')%kalman filtresli sistem cikisi Q=1 R=1 icin cizdir
hold off
title('v1 durumu icin: Q = 1 R = 1')

subplot(2,2,2)
plot(kal2v)%v1 durum Q=1 R=1000 icin cizdir
hold on
plot(sist_out2)%kalman filtresli sistem cikisi Q=1 R=1000 icin cizdir
hold off
title('v1 durumu icin: Q = 1 R = 1000')

subplot(2,2,3)
plot(kal3v)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold on
plot(sist_out3)%kalman filtresli sistem cikisi Q=1000 R=1 icin cizdir
hold off
title('v1 durumu icin: Q = 1000 R = 1')

subplot(2,2,4)
hold on
plot(sistem_out)%kalman filtresi olmayan sistem cikisi cizdir
title('Tasarlanan sistem')
hold off


% BOLUM 3

% Regulation Sistemi Tasarimi = Kontrolcu

Q = 1 * eye(4); %4x4'luk bir birim matris olsun
R = 1; 
K = lqr(sistem,Q,R);

% Olusturulan sistem matrisleri cekilir

Am = sistem.A;
Bm = sistem.B;
Cm = sistem.C;
Dm = sistem.D;

% Regulation With Close Loop System

% To make our system short we made our system like A = Am - Bm * K , B=0 ,
% C = Am - Bm * K , D=0
% Otherwise it take a lot of time

% Matlabda Regulasyon Sonrasi Sistemin Cizimi
G = ss(Am - Bm * K , zeros(size(Bm)) , Cm - Dm * K , zeros(size(Dm)));

initial(G,[5 5 5 5])
ylabel('Konum');
xlabel('t(s)');
title('Closed Loop LQR Regulator Result In Script');

% Simulinkte Cizim
model = sim('REGULATORSYSTEM')
t = model.tout;

figure(6)
regulatorsistemout = model.sist_out;
plot(regulatorsistemout)
title('Closed Loop LQR Regulator Result In Simulink')

regx1out1 = model.regx1out1;
regv1out1 = model.regv1out1;
rega1out1 = model.rega1out1;
regd1out1 = model.regd1out1;
regout = model.regout;

nonx1out = model.nonx1out;
nonv1out = model.nonv1out;
nona1out = model.nona1out;
nond1out = model.nond1out;
nlout = model.nlout;

%Regulasyon Yapilmis Sistemlerin Grafik Cizimleri

figure(7)

subplot(2,2,1)
plot(regx1out1)%v1 durum Q=1 R=1 icin cizdir
hold on
plot(regout)%regrsyonlu sistem ciktisi
title('regrsyonlu x1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi')
hold off

subplot(2,2,2)
plot(regv1out1)%v1 durum Q=1 R=1 icin cizdir
hold on
plot(regout)%regrsyonlu sistem ciktisi
title('regrsyonlu v1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi')
hold off

subplot(2,2,3)
plot(rega1out1)%a1 durumu icin Q=1 R=1 icin cizdir
hold on
plot(regout)%regrsyonlu sistem ciktisi
title('regrsyonlu a1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi')
hold off

subplot(2,2,4)
plot(regd1out1)%d1 durumu icin Q=1 R=1 icin cizdir
hold on
plot(regout)%nonlinear sistem ciktisi
title('regrsyonlu d1 durumu (Q = 1 R = 1) ve sistem ciktisi');
hold off

%Nonlineer sistemlerin graffik cizimleri

figure(8)

subplot(2,2,1)
plot(nonx1out)%v1 durum Q=1 R=1 icin cizdir
hold on
plot(nlout)%nonlinear sistem ciktisi
title('nonlinear x1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi')
hold off

subplot(2,2,2)
plot(nonv1out)%v1 durum Q=1 R=1 icin cizdir
hold on
plot(nlout)%nonlinear sistem ciktisi
title('nonlinear v1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi')
hold off

subplot(2,2,3)
plot(nona1out)%a1 durumu icin Q=1 R=1 icin cizdir
hold on
plot(nlout)%nonlinear sistem ciktisi
title('nonlinear a1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi')
hold off

subplot(2,2,4)
plot(nond1out)%d1 durumu icin Q=1 R=1 icin cizdir
hold on
plot(nlout)%nonlinear sistem ciktisi
title('nonlinear d1 durumu (Q = 1 R = 1) ve nonlinear sistem ciktisi');
hold off