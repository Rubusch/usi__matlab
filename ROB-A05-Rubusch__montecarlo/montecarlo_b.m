% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25

% cleanup
clc; close all;


% movement (for simulation)
% requires position as matlab index (betw 1 and 10)
% returns and updated index value, between 1 and 10
function [new_index] = move( steps, index, worldsize )
  if 0 == steps
    new_index = index;
    return;
  endif
  if 0 == (new_index = mod( 11+ index + steps, worldsize+1 ))
    if 0 > steps
      new_index = worldsize;
    elseif 0 <= steps
      new_index = 1;
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
  printf("position - particles\n");
  [ 0:9 ; histc(probability, 0:9) ]
  figure
  hist(probability, 0:9)
endfunction


% resampling
function [particles, weights_ng] = _resample( particles, weights )
  N = length(particles);
  weights_ng = weights;

  %% shuffle points
  dataset = [particles ; weights];
  dataset= dataset(:, randperm(length(dataset)));
  particles = dataset(1,:);
  weights = dataset(2,:);

  %% resample
  for idx=1:N % particle index
    %% working with particle index, not with position index! Anyway, handle transitive indexes.

    %% underrun
    if 1 == idx
      idx_before = N;
    else
      idx_before = idx-1;
    endif

    %% overrun
    if N == idx
      idx_after = 1;
    else
      idx_after = idx+1;
    endif

    %% convert weights into particles
    w = max( [weights( idx_before ) ; weights( idx ) ; weights( idx_after )] );

    if w == weights( idx_before )
      particles( idx ) = particles( idx_before );
      weights_ng( idx ) = weights( idx_before );

    elseif w == weights( idx_after )
      particles( idx ) = particles( idx_after );
      weights_ng( idx ) = weights( idx_after );

    endif
  endfor
endfunction



%% particle filter
%function [particles] = particlefilter( worldmap, P_prior, measurement, moves )
function [particles, weights] = particlefilter( worldmap, particles, weights, observation, moves )
  N = length( particles );

  %% weights - update
  for idx=1:N % particle indexes, not positions!!


    %% MOTION MODEL

    %% get particle position w/o jitter
    pos_worldmap = particles(idx) - mod(particles(idx), 1); % conversion to int

    %% move particles to new pose and add jitter again
    particles(idx) = move( moves, pos_worldmap+1, 10) + mod(particles(idx), 1) - 1;

    %% get index
    idx_worldmap = particles(idx) - mod(particles(idx), 1) + 1;

    if 1 == observation % robot sees a landmark
      %% handle moves and convert to an index

      if worldmap(idx_worldmap) == observation
        % if particle value is close to worldmap(idx), is around a landmark
        weights(idx) = 0.8 * weights(idx);
      else
        % if particle value is close to worldmap(idx), is around a wall
        weights(idx) = 0.2 * weights(idx);
      endif

    else % robot does not see a landmark
      if worldmap(idx_worldmap) == observation
        % if particle value is close to worldmap(idx), is around a landmark
        weights(idx) = 0.6 * weights(idx);
      else
        % if particle value is close to worldmap(idx), is around a wall
        weights(idx) = 0.4 * weights(idx);
      endif
    endif
  endfor

  % update particles
  [particles, weights] = _resample( particles, weights );
endfunction


%% test runs
function run_particle_robot( N )
  N = 100;
  worldmap    = [ 1 0 0 1 0 0 1 0 0 0 ];
  worldsize = length( worldmap );

  %% print world (just information)
  world_with_landmarks = [ 0:9; worldmap ]

  %% particles - initial
  particles = zeros(1,N);
  pos_particle = 0;
  fraction = N / 10;
  for idx=1:N
    particles(idx) = pos_particle;
    if 0 == mod(idx, fraction)
      pos_particle += 1;
    endif
  endfor

  %% weights - initial
  weights = ones(1,N);
  robot = 1;
  robot_sensing = -1;
  moves = 0;

  robot_sensing = sensing( robot, worldmap );
  [particles, weights] = particlefilter( worldmap, particles, weights, robot_sensing, moves );
  report( 1, moves, robot_sensing, particles );

  moves = 3;
  robot = move( moves, robot, worldsize);
  robot_sensing = sensing( robot, worldmap );
  [particles, weights] = particlefilter( worldmap, particles, weights, robot_sensing, moves );
  report( 2, moves, robot_sensing, particles );

  moves = 4;
  robot = move( moves, robot, worldsize);
  robot_sensing = sensing( robot, worldmap );
  particles = particlefilter( worldmap, particles, weights, robot_sensing, moves );
  report( 3, moves, robot_sensing, particles );
  printf("\n\n");
endfunction



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% START                                                                        
printf("########################################################################\n");

%% init - 10 particles
run_particle_robot( 100 );

%% init - 100 particles
run_particle_robot( 100 );

%% init - 1000 particles
run_particle_robot( 1000 );

printf( "READY.\n" );
