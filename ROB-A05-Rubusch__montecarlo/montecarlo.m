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
function [x] = f( x, v, t )
  x = 0.5 * x  +  25 * x / (1 + x^2 )  +  8 * cos( 1.2 * (t-1))  +  v;
endfunction

% nonlinear measurement function
function [y] = h( x, w )
  y = x^2/20 + w;
endfunction

% process noise
function [n] = v()
  vMean = 0; % mean
  vCov  = 1; % covariance
  n = vMean + sqrt( vCov ) * randn;
endfunction

% measurement noise
function n = w()
  wMean = 0; % mean
  wCov  = 1; % covariance
  n = wMean + sqrt( wCov ) * randn;
endfunction

% resampling
function [particles] = re_sample(xp, pdf)
  cdf = cumsum( pdf ); % cumulative sum
  diff = cdf' * ones( 1, length(pdf) ) -  ones( length(pdf), 1) * rand( 1, length(pdf) );
  diff = (diff <= 0) * 2 + diff;
  [~, idx] = min(diff);
  particles = xp(idx);
endfunction


%% particle filter
function [particles] = particlefilter( worldmap, P_prior, measurement, moves )
  % init
  N = 100;    % num of particles
  T = 10;     % num of timesteps
  x = 0.1;    % init state 
  X = x;      % initial sequence of true states 
  pCov = 2;   % initial covariance of particle distribution 

  wCov=1;     % TODO  

% p
  particles = x + sqrt( pCov ) * randn( length( x ), N ); % init particles
  xp = zeros( length( x ), N ); % init state estimate
  X_hat = []; % init sequence of state estimates

% q
  particles_weight = zeros( 1, N ); % init particle weights




  for t = 1:T
    % nonlinear discrete time system with additive noise
    x = f( x, v, t ); % true state at time t
    y = h( x, w ); % observation at time t




    % particle filtering
    for i = 1:N
      xp( i ) = f( particles( i ), v, t );  % particle prediction
      yp( i ) = h( xp( i ), w ); % prediction measurement



      d = y - yp;  % diff betw the pred measurement and observation
      particles_weight( i ) = 1 / sqrt( 2 * pi * wCov ) * exp( -d^2 / (2 * wCov) ); % asgn importance weight to each particle
    endfor
    particles_weight = particles_weight./sum( particles_weight ); % normalize the likelihood of each a priori estimate



    % the state estimate is the weighted mean of the particles
    x_hat = particles_weight * particles'; % weighted sum
    X = [ X x ];
    X_hat = [ X_hat x_hat ];     % update sequence of true states and state estimates




    particles = re_sample( xp, particles_weight ); % resampling
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
probability = [ 1 1 1 1 1 1 1 1 1 1 ];
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
probability = particlefilter( worldmap, probability, robot_sensing, moves );
report( 1, moves, robot_sensing, probability );
%figure
%hold on;
%grid("minor");
%xlabel( "Positions [steps]" );
%xlim = 10;
%ylabel( "Probability [fraction of one]" );
%title( "Localization with discrete Bayes Filter" );
%plot(0:9, probability, "c*;1st result;");


return;      


% 2nd iteration
moves = 3;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = particlefilter( worldmap, probability, robot_sensing, moves );
%report( 2, moves, robot_sensing, probability );
%plot(0:9, probability, "b*;2nd result;")


return;       


% 3rd iteration
moves = 4;
robot = move( moves, robot, worldsize);
robot_sensing = sensing( robot, worldmap );
probability = particlefilter( worldmap, probability, robot_sensing, moves );
report( 3, moves, robot_sensing, probability );
%plot(0:9, probability, "r*;final result;")
%hold off;


printf( "READY.\n" );
