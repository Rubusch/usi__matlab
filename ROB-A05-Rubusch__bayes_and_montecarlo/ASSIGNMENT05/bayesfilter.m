% ASSIGNMENT05
% Lothar Rubusch
% 2013-mar-25

% get data
%TODO

close all;

% discrete bayes localization
function bayeslocalization( data )
% for k=1; k<N do
%     p_hat(k,t) = 0;
%     for i=1; i<N do
%         p_hat(k,t) = p_hat(k,t) + p(X(t) = x(k) | u(k), X(t-1) = x(i)) p(i,t-1) % motion model
%     endfor
%     p(k,t) = p(y(t) | X(t) = x(k)) p_hat(k,t) % sensor model
% endfor
% nu = 0;
% for k=1; k<N do
%     nu=nu+p(k,t) % calculate normalization factor
% endfor
% for k=1; k<N do
%     p(k,t) = inv( nu ) p(k,t) % normalize
% endfor
% return p(k,t);
  
% TODO
endfunction


function particle_filter()
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



printf( "READY.\n" );