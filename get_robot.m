function robot = get_robot()

robot.m = 1;

robot.ro = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 2.0; 2.0; 2.0];
robot.k = [10,10,10,10,10,10,10,10,10,100,100,100];
robot.b = eye(2);

robot.r = [ sqrt(3)/2  1/2        0
            -sqrt(3)/2 1/2        0
            0          -1         0
            0.6233     -0.0000    1.0515
            -0.3117     0.5398    1.0515
            -0.3117    -0.5398    1.0515]';
robot.n_base = 3;
robot.initial_state = [0.1,0.1,0.0,0.0,0.0,0.0]';

end