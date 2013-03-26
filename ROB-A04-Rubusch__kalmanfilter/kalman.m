% ASSIGNMENT04
% Lothar Rubusch
% 2013-mar-17

printf( "\n\nASSIGNMENT04 - Robotics\n" );
printf( "Lothar Rubusch\n" );
printf( "2013-Mar-20\n\n" );


load commands.csv;
load sensed.csv;
load states.csv;

%% make clean...
close all;

%% KALMAN filter
function [states_corr] = kalman_avg(commands, sensed)
  % init by parameters
  % sensed for z
  % states for x
  % commands for u
  duration = length(sensed);
  dt = 1;

  %% constant matrices for kalman

  % transition matrix
  A = [ 0.9 ];

  % input matrix to translate (linear) the command to the states
  % control command value at a given time step
  B = [ 0.3 ];

  % measured matrix
  H = [ 1 ];

  % measure noise
  Q = [ 0.01 ];

  % measure error covariance
  R = [ 0.1 ];

  % identity matrix
  I = [ 1 ];

  % estimate covariance
  P = [ 0 ];

  % Kalman Gain matrix
  K = [];

  % auxiliary variables, calculated states and intermediate (hat)
  states_corr = [0];
  states_hat = [];
  P_hat = [];

  for k=2:duration-1
    % 1) update the new step given only the model and the current command
    % x'(k) = A * x(k-1) + B * u(k-1)
    states_hat(k) = A * states_corr(k-1) + B * commands(k-1);

    % 2) uncertainty of state of the current step, without sensor reading
    % P'(k) = A * P(k-1) * A^T + Q
    P_hat(k) = A * P(k-1) * A' + Q;

    % 3) gain factor: influence of the sensor reading to the predicted state
    % K(k) = P'(k) * H^T * (H * P'(k) * H^T + R)^-1
    K(k) = P_hat(k) * H' * inv(H * P_hat(k) * H' + R);

    % 4) correction of the state by the sensor value
    % x(k) = x'(k) + K(k) * ( z(k) - H * x'(k) )
    states_corr(k) = states_hat(k) + K(k) * ( sensed(k) - H * states_hat(k) );

    % 5) update of the state uncertainty by the sensor update
    % P(k) = (I - K(k) * H) * P'(k)
    P(k) = (I - K(k) * H) * P_hat(k);
  end
endfunction


%% MOVING AVERAGES
function [ u, v, mse ] = moving_avg( window, sensed )
  K = 1/(window+1);
  u = [ sensed(1) ];
  v = [ 0 ];
  v_hat = [ 0 ];
  for idx=2:length( sensed )
    u(idx) = u(idx-1) + K * (sensed(idx) - u(idx-1));
    v_hat(idx) = v(idx-1) + K * (sensed(idx) - u(idx))^2;
    v(idx) = (1 - K) * v_hat(idx-1);
  endfor
endfunction


%% just for fun
function [mdn] = moving_median( window, vec )
  mdn = [];
  for idx=window+1:length( vec )
    mdn(idx) = median(vec(idx-window:idx));
  endfor
endfunction


%% variance against, means may be from states.csv
function [var] = variation( vec, means )
  var_sum=0;
  for idx=1:length(vec)
    var_sum = var_sum + (vec(idx) - means(idx))^2;
  endfor
  var = var_sum / length(vec);
endfunction



%% START                                                                        



% kalman filter data
vec = kalman_avg(commands, sensed);
% variance calculation
kv = variation( vec, states );


% plot kalman filter
figure;
hold on;
plot( sensed, "b;sensed data;" );
plot( vec, "r;kalman corrected measurings;" );
grid;
xlabel( "Time [iterations]" );
ylabel( "Position [?]" );
title( "Position (sensed and kalman filtered) and negative variation" );
hold off;


% moving avg data
[u1, v1] = moving_avg( 1, sensed );
[var1] = variation( u1, states );
[u2, v2] = moving_avg( 2, sensed );
[var2] = variation( u2, states );
[u3, v3] = moving_avg( 3, sensed );
[var3] = variation( u3, states );
[u4, v4] = moving_avg( 4, sensed );
[var4] = variation( u4, states );


% plot comparison of moving avg data
figure;
grid;
hold on;
plot( sensed, "r;sensed data;" );
plot( u1, "b;window size 1;" );
plot( u2, "g;window size 2;" );
plot( u3, "m;window size 3;" );
plot( u4, "c;window size 4;" );
xlabel( "Time [iterations]" );
ylabel( "Position [?]" );
title( "Position with Moving Average Filter and different window sizes" );
hold off;


% plot variation
figure;
grid;
hold on;
plot( v1, "b;moving avg variation, window size 1;" );
plot( v2, "g;moving avg variation, window size 2;" );
plot( v3, "m;moving avg variation, window size 3;" );
plot( v4, "c;moving avg variation, window size 4;" );
xlabel( "Time [iterations]" );
ylabel( "Position [?]" );
title( "Moving Variation comparison to the actual data from states.csv" );
hold off;


% print some text
printf("1) Kalman Filter, see plot\n");
printf("2) Moving Average with 4 different window sizes, see plot\n");
printf("3) Find results of 1) and 2) plotted, find comment in README.\n");
printf("4) The Mean Squared Error, find comment in README\n");
printf( "variance of Kalman Filter: \t\t\t%.3f\n", kv );
printf( "variance of window size 1: \t\t\t%.3f\n", var1 );
printf( "variance of window size 2: \t\t\t%.3f\n", var2 );
printf( "variance of window size 3: \t\t\t%.3f\n", var3 );
printf( "variance of window size 4: \t\t\t%.3f\n", var4 );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% this was the assignment, just as appended addon for interest, I'm not sure,
% but I found the following in the context quite interesting to see
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


var_sensed = variation( sensed, states );
printf( "variance of sensed data against states.csv:\t%.3f\n", var_sensed );

%% just for fun
m1 = moving_median( 1, sensed );
mvar1 = variation( m1, states );
printf( "variance of median, window size 1: \t\t%.3f\n", mvar1 );

m2 = moving_median( 2, sensed );
mvar2 = variation( m2, states );
printf( "variance of median, window size 2: \t\t%.3f\n", mvar2 );

m3 = moving_median( 3, sensed );
mvar3 = variation( m3, states );
printf( "variance of median, window size 3: \t\t%.3f\n", mvar3 );

m4 = moving_median( 4, sensed );
mvar4 = variation( m4, states );
printf( "variance of median, window size 4: \t\t%.3f\n", mvar4 );


% intersting - an applied median filter seems to deliver better results than a mean avg filter?!


% plot variation
figure;
grid;
hold on;
plot( [var1, var2, var3, var4], "b;moving avg mean variation;" );
plot( [mvar1, mvar2, mvar3, mvar4], "g;moving median variation;" );
plot( [kv, kv, kv, kv], "r;kalman variation;" );
xlabel( "window sizes" );
ylabel( "variances" );
title( "Variation comparison of Mean, Median Filters to Kalman - all to states.csv" );
hold off;


% plot comparison mean vs median
figure;
grid;
hold on;
plot( [-4:4],[-4:4], "r");
plot( u1, m1, "b;window size 1;." );
plot( u2, m2, "g;window size 2;." );
plot( u3, m3, "m;window size 3;." );
plot( u4, m4, "c;window size 4;" );
xlabel( "moving mean" );
ylabel( "moving median" );
title( "Comparison moving mean vs moving median" );
hold off;


% plot graph
figure;
grid;
hold on;
plot( sensed, "r;sensed;");
plot( vec, "b;Kalman;" );
plot( u4, "m;mean window size 4;" );
plot( m4, "c;median window size 4;" );
xlabel( "Time [iterations]" );
ylabel( "Position [?]" );
title( "Position with Moving Mean, Median and Kalman" );
hold off;


printf("READY.\n");


