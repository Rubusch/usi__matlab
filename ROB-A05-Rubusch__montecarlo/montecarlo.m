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
%  x = 0.5 * x  +  25 * x / (1 + x^2 )  +  8 * cos( 1.2 * (t-1))  +  v;
  % move some timesteps t, and return new x; don't care about noise
    
endfunction

% nonlinear measurement function
function [y] = _measurement( x, w )
% TODO set by worldmap information  
%  y = x^2/20 + w;
    
endfunction

%% process noise
%function [n] = process_noise()
%  vMean = 0; % mean
%  process_noise_covariance  = 1; % covariance
%  n = vMean + sqrt( process_noise_covariance ) * randn;
%endfunction
%
%% measurement noise
%function n = measurement_noise()
%  wMean = 0; % mean
%  measurement_noise_covariance  = 1; % covariance
%  n = wMean + sqrt( measurement_noise_covariance ) * randn;
%endfunction

% resampling

function [particles] = _resample( particles, weights )
% TODO

  delta = mod(randn(),.01);
  N = length(particles);

  for idx=1:N % particle index
    %% working with particle index, not with position index! Anyway, handle transitive indexes.
    if 1 == idx
      idx_before = N;
    else
      idx_before = idx-1;
    endif
    %% convert weights into particles
% TODO is this correct?
    if weights( idx_before ) > weights( idx )
      particles( idx ) = delta + particles( idx_before );
    elseif weights( idx_before ) < weights( idx )
      particles( idx_before ) = delta + particles( idx );
    endif
  endfor


%% TODO what is omega(0)?
%  c = weights(1);
%  N = length(particles);
%  idx = 0;
%  for jdx=0:N-1
%    u = delta + jdx*(N-1);
%    while u > c
%      idx = idx+1;
%      if idx >= N
%        break
%      endif
%      c = c + weights(idx);
%% TODO: what has 'c' to do with the particles?
%    endwhile
%% TODO: how to generate the new particle entries?
%  endfor


%function [particles] = _resample(xp, pdf)
%  cdf = cumsum( pdf ); % cumulative sum
%  diff = cdf' * ones( 1, length(pdf) ) -  ones( length(pdf), 1) * rand( 1, length(pdf) );
%  diff = (diff <= 0) * 2 + diff;
%  [~, idx] = min(diff);
%  particles = xp(idx);
endfunction

% jitter
      
function [particle1, particle2] = _jitter( particle )
  particle1 = particle - mod(randn, 1);
  particle2 = particle + mod(randn, 1);
endfunction


%% particle filter
%function [particles] = particlefilter( worldmap, P_prior, measurement, moves )
function [particles] = particlefilter( worldmap, particles, observation, moves )
  N = length( particles );
%% TODO pass as parameter
%  N1 = 10; 
%  N2 = 100; 
%  N3 = 1000; 
%
%  % init
%  N = N1; % num of particles
%  T = 10; % num of timesteps
%
%% TODO rm - no noise  
%%  position_init = 0.1;    % init state 
%%  position = position_init;      % initial sequence of true states 
%%  pCov = 2;   % initial covariance of particle distribution 
%%  measurement_noise_covariance=1;     
%

  %% weights - initial
  weights = ones(1,N);

  %% weights - update
  for idx=1:N % particle indexes, not positions!!
    pos_worldmap = particles(idx) - mod(particles(idx), 1); % conversion to int

    if 1 == observation % robot sees a landmark
      if worldmap(pos_worldmap+1) == observation
% TODO how to connect weight to particle value?
        % if particle value is close to worldmap(idx), is around a landmark
        weights(idx) = 0.8 * weights(idx);  
      else
        % if particle value is close to worldmap(idx), is around a wall
        weights(idx) = 0.2 * weights(idx);  
      endif

    else % robot does not see a landmark
      if worldmap(pos_worldmap+1) == observation
% TODO how to connect weight to particle value?
        % if particle value is close to worldmap(idx), is around a landmark
        weights(idx) = 0.6 * weights(idx);  
      else
        % if particle value is close to worldmap(idx), is around a wall
        weights(idx) = 0.4 * weights(idx);  
      endif
    endif
  endfor


  % update particles
  particles = _resample( particles, weights );   

  % DEBUG
%  [ particles; worldmap; weights ]

return;



  particles = position_init + sqrt( pCov ) * randn( length( position_init ), N ); % init particles
  position_prediction = zeros( length(position_init), N ); % init state estimate
  position_hat = []; % init sequence of state estimates

%  particles_weight = zeros( 1, N ); % init particle weights
  particles_weight = ones( 1, N ); % init particle weights


    
  particles_ng = particles; % initial set empty    
  return;    
    


  for t = 1:T
    %% normally do it online, here: by every timestep (discrete)
%    % nonlinear discrete time system with additive noise     
    position_init = _state_transition( position_init, process_noise, t ); % true state at time t
    observation = _measurement( position_init, measurement_noise ); % observation at time t




    % particle filtering
    for idx = 1:N

      %% sample
      position_prediction( idx ) = _state_transition( particles( idx ), process_noise, t );  % particle prediction
      observation_predicted( idx ) = _measurement( position_prediction( idx ), measurement_noise ); % prediction measurement


%      d = observation - observation_predicted;  % diff betw the pred measurement and observation
      d = observation(idx) - observation_predicted(idx);  % diff betw the pred measurement and observation


      %% sensor model
      % FIXME
      % FIXME measurement_noise_covariance not properly inited...
      
      particles_weight( idx ) = 1 / sqrt( 2 * pi * measurement_noise_covariance ) * exp( -d^2 / (2 * measurement_noise_covariance) ); 
%      particles_weight( idx ) = 1 / sqrt( 2 * pi * measurement_noise_covariance ); % * exp( -d^2 / (2 * measurement_noise_covariance) ); 
                                % asgn importance weight to each particle
      
    endfor


    %% normalize the likelihood of each a priori estimate
    particles_weight = particles_weight ./ sum( particles_weight );


    % the state estimate is the weighted mean of the particles
    position_weighted_hat = particles_weight * particles'; % weighted sum
    position = [ position position_init ];
    position_hat = [ position_hat position_weighted_hat ];     % update sequence of true states and state estimates


    %% resampling
    particles_ng = _resample( position_prediction, particles_weight );
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


% TODO pass as parameter
N1 = 10; 
N2 = 100; 
N3 = 1000; 

%% init
N = N1; % num of particles
%  T = 10; % num of timesteps, not necessary - moving and sensing

%% particles - initial
particles = ones(1,N);
for idx=1:N
%    particles(idx+1) = 2*pi/10 * idx; % working on circle
  particles(idx) = idx-1; % working on positions
endfor

%% weights - initial
%weights = ones(1,N);


% 1st iteration
robot_sensing = sensing( robot, worldmap );

particles = particlefilter( worldmap, particles, robot_sensing, moves );
particles

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
