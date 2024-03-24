justpd = readmatrix("from-65_justkd.csv");
unstable = readmatrix("kd1_unstable.csv");
from65 = readmatrix("from-65_1.8_Settle.csv");
from35 = readmatrix("from35_1.2_settle.csv");
time = 0:0.1:4.2;

%%

figure
plot(0:0.1:8.0,unstable(:,1),"LineWidth",1)
xlim([0.1 8.0])
legend("Current Feedback Value")
xlabel("Time(s)")
ylabel("Current (mA)")
title('Joint 4 Current Control K_p=1, K_d=0')

figure
plot(0:0.1:5.0,from65(:,1),"LineWidth",1)
xlim([0.1 5.0])
legend("Current Feedback Value")
title('Joint 4 Current Control Initial Position Down, K_p=0.041, K_d=0.003')
xlabel("Time(s)")
ylabel("Current (mA)")

figure
plot(0:0.1:5.2,from35(:,1),"LineWidth",1)
ylim([-10 40])
xlim([0.1 5.2])
legend("Current Feedback Value")
title('Joint 4 Current Control Initial Position Up, K_p=0.041, K_d=0.003')
xlabel("Time(s)")
ylabel("Current (mA)")