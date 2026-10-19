function [dx, force, moment, iterations, updateError] = rates(x, u, gust, F, guess)
% Converge the implicit state-rate dependence in the supplied aero model.
% x: [u v w p q r phi theta psi Xe Ye Ze]'; u: [dt de da dr df]'.
% Based on the iteration in the course framework and week8_aircraft_response.
if nargin < 5, guess = zeros(12, 1); end
for iterations = 1:200
    [force, moment] = aero5560_aero(x, gust, guess, u, F);
    dx = aero5560_motion(x, force, moment, F);
    dx = dx(:);
    assert(all(isfinite(dx)), 'Non-finite state derivative.');
    updateError = norm(dx - guess, inf);
    if updateError < 1e-11, return; end
    guess = dx;
end
error('Group8:Rates', 'State-rate iteration failed to converge.');
end
