# Implementation details for the MPC project

### Model Description
The car's state *X* consists of six parameters:
- The car's position `(px,py)` and orientation `psi`,
- The car's speed magnitude `v`,
- The cross-track error `cte` and the direction error `ePsi`.

The MPC optimizes 2 control inputs for the vehicle, the steering `delta`
and acceleration/deceleration `a`. The wheels may turn up to 25 degrees in ether direction,
and the accelerator/brake pedal (combined) can go from -1.0 (full braking power) to 1.0
(pedal to the metal).

To predict the car's movement, I used the following equations:
```
px[t+1] = px[t] + v[t] * cos(psi[t]) * dt
py[t+1] = py[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = y[t] - f(x[t]) + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

where `t+1` is the next time step when the state is being reevalued/recalculated.

### Horizon (time window) discussion
My first thought when it came to this was to do some googling and math.
I wanted to design the controller to be able to drive smoothly at 100km/h (27,77m/s) on a curvy road -
so the curvy part of the track was my 'benchmark' zone. Also, I wanted to be able to predict
the movement over the next 62m of road (this was an idea I got while reading
[this article](http://www.motortrend.com/news/20-best-60-to-0-distances-recorded/) - I doubled the worst stopping
distance = 202ft = 61,5m).

Driving at 100km/h over 61,5m equals 2.25s, so I decided to go with 2,5s for the horizon.
All that was left was to optimize N and dt, trying to maximize N (or in other words, minimize dt) over the horizon.
Some of the combinations I tried:
- N=10, dt=0.25 - the optimizer fitted some crazy paths, didn't follow the road well. Hm?
- N=20, dt=0.125 - much better driving, but crazy paths appear once in a while.
Weights should be tuned...
- N=15, dt=0.166 - with good weights this speeds up the calculations while providing a dense point set.

In the end, I settled with N = 15 and dt = 0.166.
Some weight tuning had to be done to get the best result.

I decided to start with a relatively big time period between actuations at first, but quickly
recognized that the optimizer kind of works better with denser points in between actuations.
Also, a huge penalty for CTE and ePsi errors had to be introduced by weighing the cost function
in order to allow the optimizer to return decent results.

### Waypoints and coordinate system transformation
A polynomial of a third degree is fitted to waypoints on the map.
Prior to fitting, I transformed the track's waypoints (global coordinate system) presented
by the simulator to the vehicle's (local) coordinate system.
This allows for math simplification when calculating the delayed state (in 100ms),
since the local coordinates' origin is the vehicle's center (0,0).
Prior to running the optimization process (MPC algorithm), apart from the waypoint transformation, the vehicle state is
changed to reflect a 'delayed' state (100ms later) in order to provide quality inputs for navigating the car around
the track.

### Dealing with latency
Dealing with latency was probably both one of the most important and straight-forward problems
I had to solve during my work on this project. To account for latency, I chose to predict the car's state
one latency time-step ahead (e.g. 100ms in the future). I use the predicted state as a starting point
for the optimization process - that way, the optimizer takes into account the state in which the car
will be when the next actuation should be made, which was exactly what was necessary to ensure accurate
actuations for the self-driving car.
