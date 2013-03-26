%% ASSIGNMENT03
%%
%% Lothar Rubusch
%%
%% 2013-Mar-12
%%

printf( "\n\nASSIGNMENT03 - Robotics\n" );
printf( "Lothar Rubusch\n" );
printf( "2013-Mar-12\n" );

%load -ascii data.txt
load data.mat


%% KNOWN

% Our column constants. e.g., data(:,TIME) is the vector of all the times.
TIME=1;             % timestamp of the measurement, in seconds
LEFTTICKS=2;        % number of quadphase ticks for the left motor
RIGHTTICKS=3;       % number of quadphase ticks for the right motor
LEFTCURRENT=4;      % current consumption of left motor (in amps)
RIGHTCURRENT=5;     % current consumption of right motor (in amps)
LEFTPWM=6;          % PWM value for left motor ([-255,255])
RIGHTPWM=7;         % PWM value for right motor ([-255, 255])
GYRO=8;             % gyro estimate of heading (radians)

% distance between the wheels is 450mm
% wheel speeds are measure by ticks of the enconders
% for 1000 ticks the wheel moves 4.1mm - CORRECTED to 41mm
% initial position is (0,0,0)


%% EXERCISE

% number of timestamp corresponds to number of measurings
num_measurings = size( data )(1);

% axis
distance_btw_wheels = 450;

% conversion from ticks to mm
ticks_to_mm = 41 / 1000;

% right motor data
vec_rightticks = data(:,3);
vec_rightmm = ticks_to_mm * vec_rightticks;

% left motor data
vec_leftticks = data(:,2);
vec_leftmm = ticks_to_mm * vec_leftticks;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% 1) calculate the positions of a robot on a plane for the whole dataset by
%%    using only the motor tick information
x = 0;
y = 0;
Theta = 0;
vec_position = [];
mat_position = [];
cumulus = [];
for idx = 1:num_measurings
  % get motor data
  l = vec_leftmm( idx );
  r = vec_rightmm( idx );

  if r == l
    %% going straight

    % get cumulus-points
    cumulus = [ cumulus ;...
                r ]; % just any direction is fine

    % extend position matrix
    vec_orig_position = [ x ;...
                          y ;...
                          Theta ];

    vec_move_position = [ r * cos( Theta ) ;...
                          r * sin( Theta ) ;...
                          0];

    % append matrix
    vec_position = vec_move_position + vec_orig_position;

  else
    % we're turning

    % omega
    omega = ( r-l ) / (distance_btw_wheels);

    % rotation matrix
    mat_rotation = [ cos(omega) -sin(omega)  0 ;...
                     sin(omega)  cos(omega)  0 ;...
                     0           0           1 ];

    % R
    R = distance_btw_wheels * (r+l) / (2 * (r-l) );

    % get cumulus-points
    cumulus = [ cumulus ;...
                abs( omega * R ) ];

    % ICC
    vec_ICC = [ x - R * sin( Theta ) ;...
                y + R * cos( Theta ) ];

    % delta to ICC
    vec_dICC = [ [ x ; y ] - vec_ICC ;...
                  Theta ];

    % preparation
    vec_ICC = [ vec_ICC ;...
                omega ];

    % position matrix snapshot extended
    vec_position = mat_rotation * vec_dICC + vec_ICC;
  end

  % append results of iteration
  mat_position = [ mat_position  vec_position ];

  % get ready for next iteration
  x     = vec_position(1);
  y     = vec_position(2);
  Theta = vec_position(3);
end

%% RESULT: mat_position
printf("1) position matrix calculated: mat_position\n");
printf("final x [mm]\t\t= %d\n", vec_position(1));
printf("final y [mm]\t\t= %d\n", vec_position(2));
printf("final orientation\t= %f\n\n", vec_position(3));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% 2) plot the path
printf( "2) plot of path\n\n" );
subplot(3, 1, 1)
plot( mat_position(1,:) * 1/1000, mat_position(2,:) * 1/1000, ';position in m;' )
xlabel("x in m")
ylabel("y in m")


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% 3) what is the traveled distance of the robot
traveled_distance = sum( cumulus );
printf( "3) traveled distance of the robot is %d mm = %.2f m\n\n"\
       , traveled_distance, traveled_distance / 1000 );



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% 4) what is the distance from the start to the end position
vec_initialposition = mat_position(:,1);
vec_finalposition = mat_position(:,num_measurings);
direct_distance = norm( vec_finalposition(1:2,1) - vec_initialposition(1:2,1) );
printf( "4) the distance from start to end position is %d mm = %.2f m\n\n"\
       , direct_distance, direct_distance / 1000 );


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% 5) plot the calculated orientation of the robot and the gyro orientation over
%%    time in one plot with different colors. What do you observe?
printf( "5) plot of calculated orientation\n\n" );
vec_gyro = data(:,8);
vec_Theta = mat_position(3,:);

% some assumptions on the gyro...
%
% To me it seems as if the gyro when it comes to its limits on the left / right,
% simply starts over 2pi (means when the robot turns 360 degree)
%

% adjustment of the calculated REAL orientation
vec_Theta_adjusted = [];
for idx=1:num_measurings
  tmp = vec_Theta(idx);
  while tmp > 2*pi
    tmp = tmp - 2*pi;
  end
  while tmp < 0
    tmp = tmp + 2*pi;
  end
  vec_Theta_adjusted = [vec_Theta_adjusted tmp];
end


subplot(3, 1, 2)

plot( 1:num_measurings, vec_gyro, 'g;gyro data;', vec_Theta, 'r;orientation;' )
title( "raw gyro data vs calculated orientation" )
xlabel("time in steps")
ylabel("angel in rad")


subplot(3, 1, 3)
plot( 1:length( vec_gyro)\
     , vec_gyro, 'g;gyro data leveled to 0;'\
     , vec_Theta_adjusted, 'r;orientation data adjusted;' )

title( "gyro data vs adjusted orientation" )
xlabel("time in steps")
ylabel("angel in rad")


% as shown in the plot, the gyro shifts a bit from the real data
% this may be on the one side, due to inexact measurings, on the other side,
% it seems - especially in the first phase of measurings - as if the gyro has
% somehow skewed values, when the orientation is straight.
%
% This might explain that further measurings are always lower than the
% calculated orientation, this becomes clear especially with slightly adjusted
% adjusted Theta values; in a further step the slope could also identified and
% corrected.


printf( "READY.\n\n" );
