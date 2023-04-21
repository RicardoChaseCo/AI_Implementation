function vel = swarm(rr, ro, ra, x, blind_neighbors, N, dxi)
% FILE: swarm.m implements a Boids-like behavior
%
% DESCRIPTION:
% Boids-like repulsion-orientation-attraction behavior based loosely on the 
% behavior described by Couzin et al. in the Collective Memory paper. 
%
% INPUTS:
% rr - radius of repulsion
% ro - radius of orientation
% ra - radius of attraction
% x - matrix containing the pose of all the robots; x(1, ii) is the
% position of robot ii along the horizontal axis; x(2, ii) is the position
% of robot ii along the vertical axis; x(3, ii) is the heading of robot ii
% in radians. Easier alternative to dealing with radians is to use
% dxi(:,ii) instead, which is the heading or velocity of robot ii, as a
% vector
% blind_neighbors - matrix tracking the robots in a robot's blind spot
% neighbors - NxN matrix; entry (ii, jj) is 1 if agents ii and jj are
% neighbors; otherwise, entry is 0
% N - the number of robots in the swarm
% dxi - the current velocity of the robots (2 x N vector); dxi(ii, 1) is
% the velocity component along the horizontal axis, while dxi(ii, 2) is the
% velocity component along the vertical axis)
%
% OUTPUTS:
% vel - the resulting velocity of the robots (2 x N vector); vel(ii, 1) is
% the velocity component along the horizontal axis, while vel(ii, 2) is the
% velocity component along the vertical axis)
%
% TODO:
% Return the velocity (i.e., heading) vel that incorporates repulsion,
% orientation, and attracton with neighbors

%% Authors: Safwan Alam, Musad Haque - 2018
%%%%%%%%%%%%%

% dist(ii, jj) is the distance between robots ii and jj
dist = distances_from_others(x, N); 

% Random movement <-- REMOVE
% dxi = -1 + 2*rand(2, 10);

% Repulsion

% Keep a copy of the original velocity vector. You might need it to avoid 
% a scenario during orientation where robot ii is trying to align with 
% robot jj while robot jj's heading is being modified
dxi_old = dxi;

% repulsion, orientation, and attraction
repulsion = zeros(2, N);
orientation = zeros(2, N);
attraction = zeros(2, N);

for ii = 1:N
    for jj = 1:N
        if ii == jj || blind_neighbors(ii, jj)
            continue;
        end
        
        ij_dist = dist(ii, jj);
        ij_diff = x(1:2, jj) - x(1:2, ii);
        
        % Repulsion
        if ij_dist < rr
            repulsion(:, ii) = repulsion(:, ii) - ij_diff;
        % Orientation
        elseif ij_dist < ro
            orientation(:, ii) = orientation(:, ii) + dxi_old(:, jj);
        % Attraction
        elseif ij_dist < ra
            attraction(:, ii) = attraction(:, ii) + ij_diff;
        end
    end
end

dxi = repulsion + orientation + attraction;

% Return the velocity
vel = dxi;

end
