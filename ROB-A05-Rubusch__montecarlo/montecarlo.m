% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25

% cleanup
close all;


% movement (for simulation)
function [new_position] = move( steps, position, worldsize )
  if 0 == (new_position = mod( 11+ position + steps, worldsize+1 ))
    if 0 > steps
      new_position = 10;
    elseif 0 < steps
      new_position = 1;
    else
      printf("ERROR: position input was 0 and no movements\n");
      new_position = -1;
    endif
  endif
endfunction


% sensing for simulation
function [sense] = sensing( robot, worldmap )
  sense = worldmap( robot );
endfunction


% report
function report( idx, moves, sensing, probability )
  printf("%d. robot moved %d and senses ", idx, moves);
  if 1 == sensing
    printf("a LANDMARK\n");
  else
    printf("NOTHING\n");
  end
  printf("position - probability\n");
  [ 0:9; probability]
endfunction


%% aux functions

% nonlinear state transition equation
function [x] = _state_transition( x, v, t )
  x = 0.5 * x  +  25 * x / (1 + x^2 )  +  8 * cos( 1.2 * (t-1))  +  v;
endfunction

% nonlinear measurement function
function [y] = _measurement( x, w )
% TODO set by worldmap information  
  y = x^2/20 + w;
endfunction

% process noise
function [n] = process_noise()
  vMean = 0; % mean
  process_noise_covariance  = 1; % covariance
  n = vMean + sqrt( process_noise_covariance ) * randn;
endfunction

% measurement noise
function n = measurement_noise()
  wMean = 0; % mean
  measurement_noise_covariance  = 1; % covariance
  n = wMean + sqrt( measurement_noise_covariance ) * randn;
endfunction

% resampling
function [particles] = resample(xp, pdf)
  cdf = cumsum( pdf ); % cumulative sum
  diff = cdf' * ones( 1, length(pdf) ) -  ones( length(pdf), 1) * rand( 1, length(pdf) );
  diff = (diff <= 0) * 2 + diff;
  [~, idx] = min(diff);
  particles = xp(idx);
endfunction




%% particle filter
%function [particles] = particlefilter( worldmap, P_prior, measurement, moves )
function [particles_ng] = particlefilter( worldmap, particles, observation, moves )
% TODO pass as parameter
  N1 = 10; 
  N2 = 100; 
  N3 = 1000; 

  % init
  N = N1;     % num of particles
  T = 10;     % num of timesteps
  position_init = 0.1;    % init state 
  position = position_init;      % initial sequence of true states 
  pCov = 2;   % initial covariance of particle distribution 



% TODO  
  measurement_noise_covariance=1;     



  particles = position_init + sqrt( pCov ) * randn( length( position_init ), N ); % init particles
  position_prediction = zeros( length(position_init), N ); % init state estimate
  position_hat = []; % init sequence of state estimates
  particles_weight = zeros( 1, N ); % init particle weights


  particles_ng = particles; % initial set empty  



  for t = 1:T
    % nonlinear discrete time system with additive noise
    position_init = _state_transition( position_init, process_noise, t ); % true state at time t
    observation = _measurement( position_init, measurement_noise ); % observation at time t




    % particle filtering
    for idx = 1:N
      position_prediction( idx ) = _state_transition( particles( idx ), process_noise, t );  % particle prediction
      observation_predicted( idx ) = _measurement( position_prediction( idx ), measurement_noise ); % prediction measurement



      d = observation - observation_predicted;  % diff betw the pred measurement and observation
      % FIXME
      % FIXME measurement_noise_covariance not properly inited...
      
%      particles_weight( idx ) = 1 / sqrt( 2 * pi * measurement_noise_covariance ) * exp( -d^2 / (2 * measurement_noise_covariance) ); 
      particles_weight( idx ) = 1 / sqrt( 2 * pi * measurement_noise_covariance ); % * exp( -d^2 / (2 * measurement_noise_covariance) ); 
                                % asgn importance weight to each particle
      

    endfor
    particles_weight = particles_weight./sum( particles_weight ); % normalize the likelihood of each a priori estimate



    % the state estimate is the weighted mean of the particles
    position_weighted_hat = particles_weight * particles'; % weighted sum
    position = [ position position_init ];
    position_hat = [ position_hat position_weighted_hat ];     % update sequence of true states and state estimates




    particles_ng = resample( position_prediction, particles_weight ); % resampling
  endfor





%                     
%
%  worldsize = length( worldmap );
%% TODO sample(x(i) ; omega(i))
%  samples = [ P_prior ; randn(1, worldsize) ];  
%
%  % weighting factor
%  omega = zeros( 1, worldsize );
%
%  % probabilities, sensor
%  p_see = 0.8;                % P( see landmark       | landmark )
%  p_see_err = 1-p_see;        % P( don't see landmark | landmark )
%  p_notsee_err = 0.4;         % P( see landmark       | no landmark )
%  p_notsee = 1-p_notsee_err;  % P( don't see landmark | no landmark )
%
%
%
%% TODO is this needed?
%%  for i=1:worldsize
%    for j=1:worldsize
%      %% motion model
%% TODO 
%      ;
%    endfor
%
%    nu = 0;
%    for j=1:worldsize
%      %% sensor model
%      if 1 == worldmap(j)
%        % map shows a landmark, and...
%        if 1 == measurement
%          % ...a landmark sensed
%          omega(j) = p_see;
%        else
%          % ...no landmark sensed (incorrect)
%          omega(j) = p_see_err;
%        endif
%      else
%        % map has NO landmark, and...
%        if 1 != measurement
%          % ...no landmark sensed
%          omega(j) = p_notsee;
%        else
%          % ...a landmark sensed (incorrect)
%          omega(j) = p_notsee_err;
%        endif
%      endif
%      %% calculate normalization factor
%      nu += omega(j);
%    endfor % j
%
%    %% normalize
%    for j=1:worldsize
%      omega(j) *= 1/nu;
%    endfor % j
%
%    %% resampling
%% TODO check
%    jitter = randn(1,worldsize);  
%    c = omega(1);
%    u = zeros(1, worldsize);  
%%    i_next = i;  
%    i_next = 0;  
%    for j=1:worldsize
%%      u(i) = jitter + j*1/worldsize;  
%      u = jitter + j*1/worldsize;  
%%      while u(i) > c  
%      while u > c  
%        i_next++;  
%%        c += omega(i);  
%        c += omega;  
%      endwhile
%%      samples_next = union( samples, x(i) );  
%% TODO merge to samples_next
%    endfor
% TODO
%  endfor % i




% for i=1; i<k do % normally done online
%     for j=1; j<N do % for all particles
%         compute a new state x by sampling according to P(x|u(i-1,x(j)));
%         x_hat(j) = x;
%     endfor
%     nu = 0;
%     for j=1; j<N do % for all particles
%         omega(j) = P(y(i)|x(j)); % sensor model
%         nu = nu + omega(j) % calculate normalization factor
%     endfor
%     for j=1; j<N do % for all particles
%         omega(j) = inv(nu) * omega(j); % normalize
%     endfor
%     M = resampling(M);
% endfor

% TODO
endfunction



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START                                                                        

%% init
worldmap    = [ 1 0 0 1 0 0 1 0 0 0 ];
particles = [ 1 1 1 1 1 1 1 1 1 1 ];
worldsize = length( worldmap );
robot = 1;
robot_sensing = -1;
moves = 0;
% robot position in world, unknown to the robot!
%robot = mod(abs(int8(randn * 100)), worldsize)


printf("########################################################################\n");
printf("robot on position %d\n", robot);


% print world
[ 0:9; worldmap ]


% 1st iteration
robot_sensing = sensing( robot, worldmap );
particles = particlefilter( worldmap, particles, robot_sensing, moves );
report( 1, moves, robot_sensing, particles );
%figure
%hold on;
%grid("minor");
%xlabel( "Positions [steps]" );
%xlim = 10;
%ylabel( "Particles [fraction of one]" );
%title( "Localization with discrete Bayes Filter" );
%plot(0:9, particles, "c*;1st result;");


return;      


% 2nd iteration
moves = 3;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
particles = particlefilter( worldmap, particles, robot_sensing, moves );
%report( 2, moves, robot_sensing, particles );
%plot(0:9, particles, "b*;2nd result;")


return;       


% 3rd iteration
moves = 4;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
particles = particlefilter( worldmap, particles, robot_sensing, moves );
report( 3, moves, robot_sensing, particles );
%plot(0:9, particles, "r*;final result;")
%hold off;


printf( "READY.\n" );
