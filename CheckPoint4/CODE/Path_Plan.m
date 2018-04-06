% Initialize start and goal trees with the given start and goal poses
start = theta_start;
start_parent = zeros(1,1);
goal = theta_goal;
goal_parent = zeros(1,1);

% Currently the trees are not connected. Run code until they are
connected = 0;
while connected == 0 && length(start) < 250 && length(goal) < 250
    
    theta = -pi + 2*pi*rand(length(S(1,:)),1); % Sample random configuration and test for collision
    c_pose = pose_collision(theta, S, p_robot, p_obstacle, r_robot, r_obstacle);
    if c_pose == 0 % No collision found
        inS = 0;
        inG = 0;
        
        % Find closest configuration to the point in both trees
        min_start_index = find(vecnorm(theta - start) == min(vecnorm(theta - start)));
        min_start = start(:,min_start_index);
        
        min_goal_index = find(vecnorm(theta - goal) == min(vecnorm(theta - goal)));
        min_goal = goal(:,min_goal_index);
        
        % Test for straight-line path collision between both points. If any
        % path is collision-free, add the sampled configuration to that
        % tree and make the closest configuration its parent node
        start_col = path_collision(theta, min_start, S, p_robot, p_obstacle, r_robot, r_obstacle);
        if start_col == 0
            inS = 1;
            start = [start theta];
            start_parent = [start_parent min_start_index];
        end
        
        goal_col = path_collision(theta, min_goal, S, p_robot, p_obstacle, r_robot, r_obstacle);
        if goal_col == 0
            inG = 1;
            goal = [goal theta];
            goal_parent = [goal_parent min_goal_index];
        end
        
        % When sampled configuration is appended to both trees, the path is
        % connected
        if inG + inS == 2
            connected = 1;
            ls = length(start(1,:)); + 1;
            lg = length(goal(1,:)); + 1;
            
            % Construct array of connected start configurations
            qs = start(:,ls);
            a = start_parent(ls);
            while a ~= 0
                qs = [qs start(:,a)];
                a = start_parent(a);
            end
            
            % Construct array of connected goal configurations
            qg = goal(:,lg);
            b = goal_parent(lg);
            while b ~= 0
                qg = [qg goal(:,b)];
                b = goal_parent(b);
            end
            
            % Assmble the arrays into the path planning solution
            q = [fliplr(qs(:,2:length(qs(1,:)))) qg];
            disp(q);
            disp(mat2str(q)); % Display these for convenience
        end
    end
end

if connected == 0 % if the program runs too long, just give up
    fprintf('Error 404: Path not found')
end

function s = skew3(x)
    s = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end

function s = skew4(x)
    s = [skew3(x) [x(4); x(5); x(6)]; 0 0 0 0];
end

function T = find_fk(S, theta, M)
    [m,n] = size(S);
    T = 1;
    for N = 1:n
        T = T * expm(skew4(S(:,N))*theta(N));
    end
    T = T * M*m/m;
end

function c = collide_check(p1,p2,r1,r2)
    c = 0;
    if norm(p1-p2) <= r1+r2
        c = 1;
    end
end

function c = collision(p,r,n)
    c = 0;
    for i = 1:n
        for j = i+1:length(p(1,:))
            c = c + collide_check(p(:,i),p(:,j),r(i),r(j));
        end
    end
    c = min(c,1);
end

function c = pose_collision(theta, S, p_robot, p_obstacle, r_robot, r_obstacle)
    n_spheres = length(p_robot(1,:));
    pT = [p_robot;ones(1,n_spheres)];
    for j = 3:n_spheres
        pT(:,j) = find_fk(S(:,1:j-2),theta,pT(:,j));
    end
    
    spheres = [pT(1:3,:) p_obstacle];
    radii = [r_robot r_obstacle];
    c = collision(spheres,radii,length(n_spheres));
end

function c = path_collision(theta_start, theta_goal, S, p_robot, p_obstacle, r_robot, r_obstacle)
c = 0;
n_spheres = length(p_robot(1,:));
h = 0.01;
s = 0:h:1;
j = 2;
    while j < length(s)+1 && c == 0
        t = (1-s(j))*theta_start + s(j)*theta_goal;
        pR = [p_robot;ones(1,n_spheres)];
        for k = 3:n_spheres
            pR(:,k) = find_fk(S(:,1:k-2),t,pR(:,k));
        end
        
        spheres = [pR(1:3,:) p_obstacle];
        radii = [r_robot r_obstacle];
        c = collision(spheres,radii,n_spheres);
        if c ~= 0
            c = s(j);
        end
    j = j + 1;
    end
end 
