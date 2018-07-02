# mic

a small versatile effectively single-lane microscopic traffic flow
simulator for model developments and tests, or as a core in
calibration routines. Merges are modelled in a very simple way
just by adding 
vehicles into the largest gap inside the merging region with the
inflow specified in a `.rmp<n>` file (see below). Everything is text
based, there is no GUI.

## Compiling

Assume that you are on a linux system and gnu g++ is
implemented, go to ```trunk/src/``` 
and just enter 

```
make -f makefile_withLibs mic
```
When using another compiler, change the makefile
accordingly.

## Running the Simulation

The simulations are organized into `projects`. Each project has a
variable number of input and output files to be explained below. Just
run a simulation by calling

```
mic <project>
```

## The mic project

The most important project files are included in sample projects in 
`./simulations/<model>/`, e.g., `./simulations/IDM/`, and
distinguished by their extension. Every line beginning with `%` or `#`
is interpreted as a comment line.

### input files

* `.proj`  The top-level input file controlling and specifying the
  simulation project. The only file which must be present in every
  project. The order is **fixed** (I did not want to bother with xml
  parsing). Its entries are the following 

  * `choice_BCup`: Upstream boundary conditions. These can be
    controlled by a file `.BCup` (choice_BCup=0), extrapolate the
    situation near the inflow (choice_BCup=1), or periodic
    (choice_BCup=3). If a simulation with a fixed number of vehicles
    should be run, enter the "zero" condition (choice_BCup=3). There
    are also other special-purpose options. 

  * `choice_BCdown`: The same for the downstream boundary. As for the
    upstream condition, the only
    option requiring a file is choice_BCdown=0. The equivalent of
    "zero" inflow is "free" outflow (setting 2) meaning, the first driver just
    sees a free infinite road in front of her or him

  * `choice_init`: Initial conditions. You can define the initial
    position, speed, and 
    type of every vehicle by setting choice_init=0 together with a
    specification file `.ICsingle`. You can also define macroscopic
    initial conditions by specifying the initial density (setting 2)
    or density and flow (setting 3). If only the density is chosen,
    the flow will be calculated according to the (population-averaged)
    fundamental diagram.

  * The numerical parameters tmax, xmax, and dt are
    self-explaining. If you just want a simulation of a few vehicles
    with no boundaries, select choice_BCup=choice_BCdown=2,
    choice_init=0, and a very long road (large xmax).

  * `dtout_FCD` Sampling time (inverse sampling rate) of floating-car
    records which will be written for specified vehicles if the input
    file `.floatcars` exists.

  * `dtout_tseries` Sampling time for other time series such as
    stationary detector data (written for specified locations if the
    file `.detectors` exists), cross sections (written for specified
    locations if the file `.x` exists), instantaneous travel times and
    driving comfort (written if in this project file the flag
    `travelTime_output` =1),
    global instantaneous fuel consumption, and others.

  * `dxout_3D`, `dtout_3D`, `dnout_3D` Other output filters. Setting
    `dnout_3D` to values greater than 1 is useful if wanting to
    display trajectory data of big spatiotemporal simulation ranges

  * `distr_v0_T` automatically introduces some heterogeneity into each
    vehicle type by the specified relative ranges. For example, if we
    have IDM and ACC vehicles with a time gap of 2 s and 1 s,
    respectively, and `distr_v0_T=0.2`, then the time gaps will vary
    from vehicle to vehicle in the ranges [1.6 s, 2.4 s], and [0.8 s,
    1.2 s], respectively. Notice that this randomness is fixed for
    each vehicle at vehicle generation time. If you want acceleration
    noise for each vehicle over time, add the input file `.fluct` to
    the project.

* `.BCup` Specification of the upstream boundary conditions if
  `choice_BCup=0` in the `.proj` file.


## Numerical Integration

For our purposes, it
turned out that following _ballistic scheme_ is most efficient in
terms of computation load for a given precision. Its pseudo-code for
an update of the speeds _speed_ and positions _pos_ over a fixed time interval _dt_ reads

_speed(t+dt)=speed(t)+acc(t)*dt,_

_pos(t+dt)=pos(t)+speed(t)*dt+1/2*acc(t)*dt^2_,

where _acc(t)_ is the acceleration calculated by the car-following model
at the (old) time t.

However, other update schemes (simple Euler, Trapezoid, or 4th-order
Runge-Kutta are implemented as well.

## Graphics

This simulator is purely text-based and has neither GUI nor
graphics. However, in the sample projects running under Linux, there
is also a gnuplot command file generating some plots. 
## References 

[1] M. Treiber, A. Hennecke, and D. Helbing. _Congested traffic states in empirical observations and microscopic simulations._ Physical review E 62 1805-1824 (2000). [Link](http://journals.aps.org/pre/pdf/10.1103/PhysRevE.62.1805), [Preprint](http://arxiv.org/abs/cond-mat/0002177)

[2] M. Treiber and A. Kesting. [_Traffic Flow Dynamics, Data, Models and Simulation_](http://www.traffic-flow-dynamics.org). Springer 2013. [Link](http://www.springer.com/physics/complexity/book/978-3-642-32459-8)

[3] A. Kesting, M. Treiber, and D. Helbing. _General lane-changing model MOBIL for car-following models_.   Transportation Research Record, 86-94 (2007). [Paper](http://www.akesting.de/download/MOBIL_TRR_2007.pdf)
    
[4] A. Kesting, M. Treiber, and D. Helbing. _Enhanced intelligent driver model to access the impact of driving strategies on traffic capacity_. Philosophical Transactions of the Royal Society A, 4585-4605 (2010). [Preprint](http://arxiv.org/abs/0912.3613)
    
[5] M. Treiber, and A. Kesting. An open-source microscopic traffic
simulator.     IEEE Intelligent Transportation Systems Magazine, 6-13
(2010). [Preprint](http://arxiv.org/abs/1012.4913)

[6] M. Treiber and V. Kanagaraj.
Comparing Numerical Integration Schemes for Time-Continuous Car-Following Models
Physica A: Statistical Mechanics and its Applications 419C, 183-195
DOI 10.1016/j.physa.2014.09.061 (2015).
[Preprint](http://arxiv.org/abs/1403.4881)
