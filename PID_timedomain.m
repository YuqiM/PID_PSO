clear all vars
clc
close all force
%% This script is to run a time domian simulation to use PID controller to provide feedback control of the photonics accelerometer
%% This is to use PSO(Particle Swarm Optimization) to determine the optimized set of PID parameters
num_par = 64;%number of particles
pb = 0.1; %cognitive coefficient
sf = 0.2; %social coefficient 
NN = round(num_par^(1/3)); %Innitialization of particle locations, # of particles along each dimension
Kp_meta = linspace(0,1,NN); %1st dimension is set to the proportional gain of the controller
Ki_meta = linspace(2e4,1e5,NN); %2nd dimension is to set the integral gain of the controller
Kd_meta = linspace(-2e-9,2e-9,NN); %3rd dimension is the differential gain of PID
flag = 1;
for ii = 1:NN
    for jj = 1:NN
       for kk = 1:NN
           Locs(flag,1) = Kp_meta(ii);
           Locs(flag,2) = Ki_meta(jj);
           Locs(flag,3) = Kd_meta(kk);
           flag=flag+1;
       end
    end
end

Vel = zeros(num_par,3);%Innitializing the velocity
figure
for kk = 1:50 %The number of time steps
    tic
    for jj =1:num_par

        Kp = Locs(jj,1); %Unitless
        Ki = Locs(jj,2); %Unit [1/s]
        Kd = Locs(jj,3); %UNIT [s]
        fs = 4e5; %Sampling Frequency of Simulation
        fa = 1e2; %frequency of the external stimulus
        dt = 1/fs; %Time Step of Simulation
        t = 0:dt:10*1/fa; %Time Length Definition
        tao = 17e-6; %Time Delay is 17us
        K = 1; %Phase sensitivity of the Interferometer Device
        phi_ini = 3*pi/2-0.5*1*pi/8; %Innitial Phase of the Device
        Yo = 1; %Setpoint of the PID controller
        error_ini = 0; %innitial error signal
        Y_ini = Yo*(1+cos(K*error_ini+phi_ini));
        stimuli_phi = 1*2*pi*sin(2*pi*fa*t);
        %% Running the Simulation
        ki_accu = 0;
        Y_curr = Y_ini;
        for ii=1:length(t)

            error(ii) = Yo-Y_curr; %Calculate the error input to PID controller
            error_kp(ii) = Kp*error(ii); %Calculate the proportionate error
            error_ki(ii) = ki_accu+Ki*error(ii)*dt; %Calculate the integrate error
            if ii == 1
                error_kd(ii) = 0; %Innitializing the differentiate error
            else
                error_kd(ii) = Kd*1/dt*(error(ii)-error(ii-1)); %Calculate the differentiate error
            end
            ki_accu = ki_accu+Ki*error(ii)*dt; %Calculate the integral error
            error_tot = error_kp+error_ki+error_kd; %Sum the total errors
            try
                Y_curr = Yo*(1+cos(K*error_tot(ii-round(tao/dt))+phi_ini+stimuli_phi(ii))); %This will give error if the time has not propagate for time tau
                Y(ii) = Y_curr; %The output Y keeps getting influenced by PID control after tau
            catch
                %disp(strcat('Phase has yet to accumulate after',{' '},num2str(ii*dt),{' '},'s'))
                Y(ii) = Yo*(1+cos(K*error_ini+phi_ini+stimuli_phi(ii))); %Output Y keeps its value before tau.
            end

        end
        err_opt(jj) = max(Y(round(length(t)/2):end))-min(Y(round(length(t)/2):end)); %The cost function - defined as the peak to peak error of the PID process
    end
    [m(kk),n(kk)] = min(err_opt); %Calculate the global best
   
    Locs_store(kk,1:num_par,1:3) = Locs; %Store every particle's locations at each time frame
    figure(1)
    scatter3(Locs(:,1),Locs(:,2),Locs(:,3));
    xlabel('Kp');
    ylabel('Ki');
    zlabel('kd');
    hold on
    if kk==1
        opt_value = m(kk); %Innitializing the global best value
        opt_num = n(kk); %Identifying the global best particle
        Gbest = opt_value; %Record the global best
        Gbest_num = opt_num; %Record the global best particle
        Gbest_PID(1) = Locs(opt_num,1); %Record the global best coordinate X
        Gbest_PID(2) = Locs(opt_num,2); %Record the global best coordinate Y
        Gbest_PID(3) = Locs(opt_num,3); %Record the global best coordinate Z
        Pbest_error = err_opt; %Innitializing the Personal best cost function result
        Pbest_PID = Locs; %Innitializing the Personal best coordinates

    else
        for jj =1:num_par %For updating personal best
            if err_opt(jj)<=Pbest_error(jj) %if the cost function is lower
                Pbest_error(jj) = err_opt(jj); %Now update the personal best cost function result
                Pbest_PID(jj,:) = Locs(jj,:); %Update the personal best coordinate
            end
        end

        if m(kk)<opt_value %For updating the global best result
            opt_value =  m(kk); %Update the global best cost function
            opt_num = n(kk); %Update the global best particle number
            Gbest = opt_value; %Record the global best cost function
            Gbest_num = opt_num; %Record the global best particle number
            Gbest_PID(1) = Locs(opt_num,1); %Record the Global best coordinate - X
            Gbest_PID(2) = Locs(opt_num,2); %Record the Global best coordinate - Y
            Gbest_PID(3) = Locs(opt_num,3); %Record the Global best coordinate - Z
            vel = repmat(Locs(opt_num,:),num_par,1)-Locs; %Calculate the social velocity
            velpb = Pbest_PID - Locs; %Calculate the cognitive velocity
            Locs = Locs+vel*sf+velpb*pb; %Move the particles to new locations
        else
            Gbest = opt_value; %Record the global best cost function
            Gbest_num = opt_num; %Record the global best particle number
            Gbest_PID(1) = Locs(opt_num,1); %Record the Global best coordinate - X
            Gbest_PID(2) = Locs(opt_num,2); %Record the Global best coordinate - Y
            Gbest_PID(3) = Locs(opt_num,3); %Record the Global best coordinate - Z
            velpb = Pbest_PID - Locs; %Calculate the cognitive velocity
            vel = repmat(Locs(opt_num,:),num_par,1)-Locs; %Calculate the social velocity
            Locs = Locs+vel*sf+velpb*pb; %Move the particles to new locations
            
        end
    end
    scatter3(Locs(:,1),Locs(:,2),Locs(:,3));

    t1 = toc


    figure(2)

    subplot(60,3,3*(kk-1)+1)
    histogram(Locs_store(kk,:,1),10);
    xlabel('Kp values')
    ylabel('# of particles')
    title(strcat(num2str(kk),'# of movement'));
    axis([-0.3 1.1 -inf inf])

    subplot(60,3,3*(kk-1)+2)
    histogram(Locs_store(kk,:,1),10);
    axis([1e3 0.7e6 -inf inf])
    xlabel('Ki values')
    ylabel('# of particles')
    title(strcat(num2str(kk),'# of movement'));

    subplot(60,3,(kk-1)*3+3)
    histogram(Locs_store(kk,:,1),10);
    xlabel('Kd values')
    ylabel('# of particles')
    title(strcat(num2str(kk),'# of movement'));
    axis([-1e-5 2e-5 -inf inf])


end
hold off
legend('0','1','2','3','4','5','6','7','8','9','10');
for ii = 1:(length(Locs_store(:,1,1)))
    Kp_store(ii) = Locs_store(ii,n(ii),1);
    Ki_store(ii) = Locs_store(ii,n(ii),2);
    Kd_store(ii) = Locs_store(ii,n(ii),3);
end

figure
subplot(311)
plot(Kp_store,'-o');
xlabel('# of Movements');
ylabel('Kp')
title('Optimized Kp');
pedit
subplot(312)
plot(Ki_store,'-o');
xlabel('# of Movements');
ylabel('Ki')
title('Optimized Ki');
pedit
subplot(313)
plot(Kd_store,'-o');
xlabel('# of Movements');
ylabel('Kd')
title('Optimized Kd');
pedit

figure
subplot(3,4,1)
histogram(Locs_store(1,:,1),10);
xlabel('Kp values')
ylabel('# of particles')
title('1 movement')
subplot(3,4,2)
histogram(Locs_store(10,:,1),10);
xlabel('Kp values')
ylabel('# of particles')
title('10 movement')
subplot(3,4,3)
histogram(Locs_store(20,:,1),10);
xlabel('Kp values')
ylabel('# of particles')
title('20 movement')
subplot(3,4,4)
histogram(Locs_store(40,:,1),10);
xlabel('Kp values')
ylabel('# of particles')
xlabel('Kp values')
ylabel('# of particles')
title('40 movement')
subplot(3,4,5)
histogram(Locs_store(1,:,2),10);
xlabel('Ki values')
ylabel('# of particles')
title('1 movement')
subplot(3,4,6)
histogram(Locs_store(10,:,2),10);
xlabel('Ki values')
ylabel('# of particles')
title('10 movement')
subplot(3,4,7)
histogram(Locs_store(20,:,2),10);
xlabel('Ki values')
ylabel('# of particles')
title('20 movement')
subplot(3,4,8)
histogram(Locs_store(40,:,2),10);
xlabel('Ki values')
ylabel('# of particles')
xlabel('Ki values')
ylabel('# of particles')
title('40 movement')

subplot(3,4,9)
histogram(Locs_store(1,:,3),10);
xlabel('Kd values')
ylabel('# of particles')
title('1 movement')
subplot(3,4,10)
histogram(Locs_store(10,:,3),10);
xlabel('Kd values')
ylabel('# of particles')
title('10 movement')
subplot(3,4,11)
histogram(Locs_store(20,:,3),10);
xlabel('Kd values')
ylabel('# of particles')
title('20 movement')
subplot(3,4,12)
histogram(Locs_store(40,:,3),10);
xlabel('Kd values')
ylabel('# of particles')
xlabel('Kd values')
ylabel('# of particles')
title('40 movement')

%% Parameter Innitialization
Kp = Gbest_PID(1);%1*mean(Locs(:,1));
Ki = Gbest_PID(2);%1*mean(Locs(:,2));
Kd = Gbest_PID(3);%mean(Locs(:,3));
Tu = 5e-6; %closed loop tuning parameter concerning "I&D" considering phase error is pi/16
Ku = 1.3; %closed loop tuning parameter of "P"
%Kp = 0*Ku/5; %Unitless
%Ki = 1.8/(4*Tu); %Unit [1/s]
%Kd = 0*Tu/10000; %UNIT [s]
fs = 4e5; %Sampling Frequency of Simulation
fa = 1e2; %frequency of the external stimulus
dt = 1/fs; %Time Step of Simulation
t = 0:dt:30*1/fa; %Time Length Definition
tao = 17e-6; %Time Delay is 17us
K = 1; %Phase sensitivity of the Interferometer Device
phi_ini = 3*pi/2-0.5*1*pi/8; %Innitial Phase of the Device
Yo = 1; %Setpoint of the PID controller
error_ini = 0; %innitial error signal
Y_ini = Yo*(1+cos(K*error_ini+phi_ini));
stimuli_phi = 1*2*pi*sin(2*pi*fa*t);
%% Running the Simulation
ki_accu = 0;
Y_curr = Y_ini;
for ii=1:length(t)

    error(ii) = Yo-Y_curr;
    error_kp(ii) = Kp*error(ii);
    error_ki(ii) = ki_accu+Ki*error(ii)*dt;
    if ii == 1
        error_kd(ii) = 0;
    else
        error_kd(ii) = Kd*1/dt*(error(ii)-error(ii-1));
    end
    ki_accu = ki_accu+Ki*error(ii)*dt;
    error_tot = error_kp+error_ki+error_kd;
    try
        Y_curr = Yo*(1+cos(K*error_tot(ii-round(tao/dt))+phi_ini+stimuli_phi(ii)));
        Y(ii) = Y_curr;
    catch
        disp(strcat('Phase has yet to accumulate after',{' '},num2str(ii*dt),{' '},'s'))
        Y(ii) = Yo*(1+cos(K*error_ini+phi_ini+stimuli_phi(ii)));
    end

end
figure
subplot(221)
plot(t,error_kp);
xlabel('Time [s]');
ylabel('Amplitude');
title('Proportional Error - "P" ');
pedit
subplot(222)
plot(t,error_ki);
xlabel('Time [s]');
ylabel('Amplitude');
title('Integral Error - "I" ');
pedit
subplot(223)
plot(t,error_kd);
xlabel('Time [s]');
ylabel('Amplitude');
title('Differential Error - "D" ');
pedit
subplot(224)
plot(t,error_tot);
hold on
plot(t,stimuli_phi)
hold off
legend('total error signal','stimulus signal')
xlabel('Time [s]');
ylabel('Amplitude');
title('total Error - "PID" ');
pedit

figure
plot(t,Y);
axis([-inf inf 0 2]);
xlabel('Time [s]');
ylabel('Output of the Photodiode [V]')
pedit


disp(strcat('p2p oscillation is',{' '},num2str(max(Y(round(length(t)/2):end))-min(Y(round(length(t)/2):end)))))





figure
plot(Locs_store(:,30,1),'o')


