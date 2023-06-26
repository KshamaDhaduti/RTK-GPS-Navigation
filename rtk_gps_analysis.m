bag1=rosbag("lab2_open_stat.bag");
bag2=rosbag("lab2_open_moving.bag");
bag3=rosbag("lab2_obstructed_stat2.bag");
bag4=rosbag("lab2_obstructed_moving.bag");
bsel1=select(bag1,"Topic","full_gps");
bsel2=select(bag2,"Topic","full_gps");
bsel3=select(bag3,"Topic","full_gps");
bsel4=select(bag4,"Topic","full_gps");
msgStruct1=readMessages(bsel1,'DataFormat','struct');
msgStruct2=readMessages(bsel2,'DataFormat','struct');
msgStruct3=readMessages(bsel3,'DataFormat','struct');
msgStruct4=readMessages(bsel4,'DataFormat','struct');
utm_easting_open_stat=cellfun(@(m) double(m.UtmEasting),msgStruct1);
utm_easting_open_mov=cellfun(@(m) double(m.UtmEasting),msgStruct2);
utm_easting_obs_stat=cellfun(@(m) double(m.UtmEasting),msgStruct3);
utm_easting_obs_mov=cellfun(@(m) double(m.UtmEasting),msgStruct4);
utm_northing_open_stat=cellfun(@(m) double(m.UtmNorthing),msgStruct1);
utm_northing_open_mov=cellfun(@(m) double(m.UtmNorthing),msgStruct2);
utm_northing_obs_stat=cellfun(@(m) double(m.UtmNorthing),msgStruct3);
utm_northing_obs_mov=cellfun(@(m) double(m.UtmNorthing),msgStruct4);
%{
alt_open_stat=cellfun(@(m) double(m.Altitude),msgStruct1);
alt_open_mov=cellfun(@(m) double(m.Altitude),msgStruct2);
alt_obs_stat=cellfun(@(m) double(m.Altitude),msgStruct3);
alt_obs_mov=cellfun(@(m) double(m.Altitude),msgStruct4);
%}
subplot(2,1,1)
plot(utm_easting_open_stat-min(utm_easting_open_stat),utm_northing_open_stat-min(utm_northing_open_stat),'r+')
title("RTK GPS Data Plot for a stationary point's readings in an open area")
xlabel("UTM Easting (metres)")
ylabel("UTM Northing (metres)")
subplot(2,1,2)
plot(utm_easting_obs_stat-min(utm_easting_obs_stat),utm_northing_obs_stat-min(utm_northing_obs_stat),'r+')
title("RTK GPS Data Plot for a stationary point's readings in a partially occluded area")
xlabel("UTM Easting (metres)")
ylabel("UTM Northing (metres)")
%{
hold on 
p=polyfit(utm_easting_lin-min(utm_easting_lin),utm_northing_lin-min(utm_northing_lin),1);
f=polyval(p,utm_easting_lin-min(utm_easting_lin));
plot(utm_easting_lin-min(utm_easting_lin),f,'b-')
legend("Linear motion data","Best fit line for linear motion data")
hold off
%}
figure
subplot(2,2,1)
plot(utm_easting_open_stat-min(utm_easting_open_stat),'r+')
hold on
p=polyfit([1:length(utm_easting_open_stat)],utm_easting_open_stat-min(utm_easting_open_stat),1);
f_utm_east_stat=polyval(p,[1:length(utm_easting_open_stat)]);
plot(f_utm_east_stat,'b')
legend("UTM Easting for open area stationary point data","Best fit line for UTM Easting for open area stationary point data")
hold off
title("Stationary point UTM Easting Readings in the open space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)")
subplot(2,2,2)
plot(utm_easting_obs_stat-min(utm_easting_obs_stat),'r+')
title("Stationary point UTM Easting Readings in the partially occluded space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)") 
%{
hold on
p=polyfit([1:length(utm_easting_obs_stat-min(utm_easting_obs_stat))],utm_easting_obs_stat-min(utm_easting_obs_stat),1);
f=polyval(p,[1:length(utm_easting_obs_stat-min(utm_easting_obs_stat))]);
plot(f,'b')
legend("UTM Easting for straight line motion data","Best fit line for UTM Easting for straight line motion data")
hold off
%}
subplot(2,2,3)
plot(utm_northing_open_stat-min(utm_northing_open_stat),'r+')
hold on
p=polyfit([1:length(utm_northing_open_stat)],utm_northing_open_stat-min(utm_northing_open_stat),1);
f_utm_east_stat=polyval(p,[1:length(utm_northing_open_stat)]);
plot(f_utm_east_stat,'b')
legend("UTM Northing for open area stationary point data","Best fit line for UTM Northing for open area stationary point data")
hold off
title("Stationary point UTM Northing Readings in the open space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)")
subplot(2,2,4)
plot(utm_northing_obs_stat-min(utm_northing_obs_stat),'r+')
title("Stationary point UTM Northing Readings in the partially occluded space")
xlabel("Time (seconds)")
ylabel("UTM Easting (metres)")
%{
figure
subplot(2,1,1)
plot(alt1,'r+')
title("Stationary point altitude readings")
xlabel("Time(seconds)")
ylabel("Altitude(metres)")
hold on
p=polyfit([1:length(alt1)],alt1,1);
f=polyval(p,[1:length(alt1)]);
plot(f,'b')
legend("Stationary point altitude data","Best fit line for Stationary point altitude data")
hold off
subplot(2,1,2)
plot(alt2,'r+')
title("Linear motion altitude readings")
xlabel("Time(seconds)")
ylabel("Altitude(metres)")
hold on
p=polyfit([1:length(alt2)],alt2,1);
f=polyval(p,[1:length(alt2)]);
plot(f,'b')
legend("Linear motion altitude data","Best fit line for Linear motion altitude data")
hold off
%}
figure
subplot(2,1,1)
% plot(utm_easting_open_mov-min(utm_easting_open_mov),utm_northing_open_mov-min(utm_northing_open_mov),'r+')
x = utm_easting_open_mov-min(utm_easting_open_mov);
y = utm_northing_open_mov-min(utm_northing_open_mov);
mux = mean(x);
muy = mean(y);
[Theta,R] = cart2pol(x- mux,y-muy);
[Theta,ind] = sort(Theta);
R = R(ind);
n = numel(x);
M = 6;
A = [ones(n,1),sin(Theta*(1:M)),cos(Theta*(1:M))];
coeffs = A\R;
Rhat = A*coeffs;
hold on
[xhat,yhat] = pol2cart(Theta,Rhat);
l1=xhat + mux;
k1=yhat + muy;
plot(x,y,'b+',l1,k1,'.r')
l1=circshift(l1,-327);
k1=circshift(k1,-322);
title("RTK GPS Data Plot for a structured path points' readings in an open area")
xlabel("UTM Easting (metres)")
ylabel("UTM Northing (metres)")
legend("UTM Data for open area structured path points","Best fit discrete curve for Data for open area structured path points")
for i=1:397
    err_utm_easting_open_mov(i)=abs(utm_easting_open_mov(i)-min(utm_easting_open_mov)-l1(i));
    err_utm_northing_open_mov(i)=abs(utm_northing_open_mov(i)-min(utm_northing_open_mov)-k1(i));
    ++i;
end
err_utm_open_mov=((err_utm_easting_open_mov.^(2))+(err_utm_northing_open_mov.^(2))).^(0.5);
%{
err_utm_easting_open_mov=abs(utm_easting_open_mov-min(utm_easting_open_mov)-l);
err_utm_northing_open_mov=abs(utm_northing_open_mov-min(utm_northing_open_mov)-k);
err_utm_open_mov=((err_utm_easting_open_mov.^(2))+(err_utm_northing_open_mov.^(2))).^(0.5);
%}

subplot(2,1,2)
% plot(utm_easting_open_mov-min(utm_easting_open_mov),utm_northing_open_mov-min(utm_northing_open_mov),'r+')
x = utm_easting_obs_mov-min(utm_easting_obs_mov);
y = utm_northing_obs_mov-min(utm_northing_obs_mov);
mux = mean(x);
muy = mean(y);
[Theta,R] = cart2pol(x- mux,y-muy);
[Theta,ind] = sort(Theta);
R = R(ind);
n = numel(x);
M = 6;
A = [ones(n,1),sin(Theta*(1:M)),cos(Theta*(1:M))];
coeffs = A\R;
Rhat = A*coeffs;
hold on
[xhat,yhat] = pol2cart(Theta,Rhat);
l2=xhat + mux;
k2=yhat + muy;
plot(x,y,'b+',l2,k2,'.r')
l2=circshift(l2,-228);
k2=circshift(k2,-157);
title("RTK GPS Data Plot for a structured path points' readings in a partially occluded area")
xlabel("UTM Easting (metres)")
ylabel("UTM Northing (metres)")
legend("UTM Data for a partially occluded area's structured path points","Best fit discrete curve for a partially occluded area's structured path points")
for i=1:397
    err_utm_easting_obs_mov(i)=abs(utm_easting_obs_mov(i)-min(utm_easting_obs_mov)-l2(i));
    err_utm_northing_obs_mov(i)=abs(utm_northing_obs_mov(i)-min(utm_northing_obs_mov)-k2(i));
    ++i;
end
err_utm_obs_mov=((err_utm_easting_obs_mov.^(2))+(err_utm_northing_obs_mov.^(2))).^(0.5);

figure
subplot(2,1,1)
plot(err_utm_open_mov)
title("Absolute values of open area structured motion UTM data error values from best fit line values")
xlabel("Time (seconds)")
ylabel("Absolute value of error(centimetres)")
subplot(2,1,2)
plot(err_utm_obs_mov)
title("Absolute values of partially occluded area structured motion UTM data error values from best fit line values")
xlabel("Time (seconds)")
ylabel("Absolute value of error(centimetres)")
%{
utm_east_stat_err=f_utm_east_stat-utm_easting_stat;
utm_north_stat_err=f_utm_north_stat-utm_northing_stat;
figure
subplot(2,1,1)
plot(abs(utm_east_stat_err-min(utm_east_stat_err)),'o')
title('Absolute values of UTM Easting stationary point error values from best fit line values')
xlabel('Time(seconds)')
ylabel('Absolute value of error(metres)')
subplot(2,1,2)
plot(abs(utm_north_stat_err-min(utm_north_stat_err)),'o')
title('Absolute values of UTM Northing stationary point error values from best fit line values')
xlabel('Time(seconds)')
ylabel('Absolute value of error(metres)')

mean_alt_stat=mean(alt1);
fprintf("Mean of stationary point data altitude:%f",mean_alt_stat)
mean_alt_lin=mean(alt2);
fprintf("\nMean of linear motion data altitude:%f",mean_alt_lin)

mean_utm_easting_stat=mean(utm_easting_stat)
std_utm_easting_stat=std(utm_easting_stat)
mean_utm_easting_lin=mean(utm_easting_lin)
std_utm_easting_lin=std(utm_easting_lin)
min_utm_easting_stat=min(utm_easting_stat)
max_utm_easting_stat=max(utm_easting_stat)
min_utm_easting_lin=min(utm_easting_lin)
max_utm_easting_lin=max(utm_easting_lin)
range_utm_easting_lin=max(utm_easting_lin)-min(utm_easting_lin)
range_utm_easting_stat=max(utm_easting_stat)-min(utm_easting_stat)

mean_utm_northing_stat=mean(utm_northing_stat)
std_utm_northing_stat=std(utm_northing_stat)
mean_utm_northing_lin=mean(utm_northing_lin)
std_utm_northing_lin=std(utm_northing_lin)
min_utm_northing_stat=min(utm_northing_stat)
max_utm_northing_stat=max(utm_northing_stat)
min_utm_northing_lin=min(utm_northing_lin)
max_utm_northing_lin=max(utm_northing_lin)
range_utm_northing_lin=max(utm_northing_lin)-min(utm_northing_lin)
range_utm_northing_stat=max(utm_northing_stat)-min(utm_northing_stat)
figure
plot(alt1)
hold on
plot(alt2)
legend("Stationary point altitude data","Linear motion altitude data")
title("Stationary point and linear motion altitude data plot")
xlabel("Time")
ylabel("Altitude")
hold off
%}