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

### Top-level input files

The following two files are the only ones that must be present in
every `mic` project.

* `.proj`  The top-level input file controlling and specifying the
  simulation project. This and the `.heterog` file are the only ones
  which must be present in every 
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
    locations if the file `.detectors` exists), instantaneous travel times and
    driving comfort (written if in this project file the flag
    `travelTime_output` =1),
    global instantaneous fuel consumption, and others.

  * `dxout_3D`, `dtout_3D`, `dnout_3D` Other output filters. Setting
    `dnout_3D` to values greater than 1 and `choice_output=1` or=2 is
    useful if wanting to 
    display trajectory data (written in the file `.micdat` of big
    spatiotemporal simulation ranges 

  * `distr_v0_T` automatically introduces some heterogeneity into each
    vehicle type by the specified relative ranges. For example, if we
    have IDM and ACC vehicles with a time gap of 2 s and 1 s,
    respectively, and `distr_v0_T=0.2`, then the time gaps will vary
    from vehicle to vehicle in the ranges [1.6 s, 2.4 s], and [0.8 s,
    1.2 s], respectively. Notice that this randomness is fixed for
    each vehicle at vehicle generation time. If you want acceleration
    noise for each vehicle over time, add the input file `.fluct` to
    the project.

* `.heterog` Specification of the vehicle fleet. Mandatory for each
  project. Each line describes a
  certain vehicle-driver type, i.e., a certain car-following
  model. Consequently,  a simulation with identical drivers
  and vehicles is prescribed by a single data line. Each data line
  includes  three entries:
 
  * column 1: fraction: The fraction of the corresponding
    vehicle-driver type when populating the simulation by the initial
    conditions, open boundary conditions, or by the onramp flow. If
    the cumulative 
    fraction exceeds 1, the corresponding vehicle types are ignored,
    if the sum is less than 1, the last data line fills the gap to
    100%, if there is only one data line, this column is
    irrelevant. *Notice*: This fraction is overridden by the
    microscopic 
    initial conditions, see `.ICsingle`. Using `.ICsingle`, also
    vehicle types exceeding a cumulative 100% can be introduced into
    the simulation.

  * column 2: an integer denoting a certain car-following model, e.g.,
    0 for the IDM, 3 for the OVM, 7 for the FVDM, and 10 for the Gipps
    model. A 
    list of allowed values and associated models is given in the
    comment section in the header of this file.

  * column 3: an integer denoting a certain parametrisation inside a
    given model. For example, model number 0 and set number 2 will
    read the IDM (=model 0) parameters from the file `.IDM2`

    *Notice*
    another sort of heterogeneity can be realized with the
    `distr_v0_T` entry of the `proj` file.


### Model specification

This is the *Unique selling point* of the simulator `mic`: The
generality in specifying several different models in one and the same 
simulation. Even mixing of discrete CA-like and time-continuous models
is allowed.

* `.<MOD><n>` Parameter specification file of the chosen car-following
  model, e.g., `.IDM1`, `.OVM3`, or `.BIDM1` (Brownian IDM).
  `<MOD>` is a model abbreviation consiting of two to four
  uppercase letters associated to the model numbers as follows:
  0=IDM, 1=VW, 2=HDM, 3=FVDM, 4=FPE, 5=HSM, 6=VDT, 7=FVDM, 9=CDDA,
  10=GIP, 11=KCA, 12=PT, 13=ASGM, 14=ADAS, 16=CACC, 17=PCF, 18=LCM,
  19=BIDM.  

  *Notice*: that the OVM is a subset of the full-velocity
  difference model FVDM, therefore, there is no .OVM extension.

  *Notice*: Some models come in different variants such as the
  OVM/FVDM and the variant is selected by a `choice_variant` entry in
  the parameter file. For example, in the `.FVDM` files,
  `choice_variant=0` denotes the original OV function based on tanh
  functions while `choice_variant=1` denotes an OV function making up
  a triangular fundamental diagram. 

  `<n>` is the parameter set number. The relevant model parameter files
  are controlled by  the `.heterog` file. An error is produced if the
  required parameter files do not exist. 

  *Notice*: In case of meta-models such
  as the human-driver model (HDM, model number=2), two parameter files
  are needed: The `.HDM<n>` file for the parameters of the meta-model,
  and the `.IDM<n>` file for the underlying base car-following model (at
  present, only the IDM is implemented as a base model)


### Initial and boundary conditions

This section specifies the traffic demand and its spatiotemporal
change. 

* `.IC` Macroscopic initial conditions. This file consists of three
  columns: Location x, density rho, and flow Q. At the points given by
  an x value, the corresponding rho and Q values are taken. Otherwise,
  they are interpolated linearly or extrapolated by a constant equal
  to the values at the minimum/maximum x value. The fleet structure is
  controlled by the `.heterog` file. Depending on the `choice_init`
  setting in the `.proj` file, only the density, flow and density, or
  none are used. 

* `.ICsingle` Microscopic initial conditions. The vehicles are
  specified individually by their initial position, speed, and type.
  The type index is a number 0,1,2,... denoting the first, second,
  third,... data line of the `.heterog` file. So, if the `heterog`
  file reads

  ```
  % fraction    modelNumber    setNumber
  1             2              1
  0             0              2 # blocking obstacle
  ```

  and the `.ICsingle` file contains

  ```
  %x(m)   v(m/s)  type(0=first line of .heterogen file, 1=second line,...)
  10      0       0
  20      0       0
  1100    0       1   # blocking obstacle
  ```
  standing HDM1 vehicles are set at x=10m and x=20m, and a standing IDM2
  vehicle is set at x=1100m (lookin into the .IDM2 specification of the
  provided startStop projects, one
  observes that the desired speed v0=0, so this vehicle represents, in
  fact, the stopping line of a red traffic light)

* `.BCup` Specification of the (macroscopic) upstream boundary conditions if
  `choice_BCup=0` is set in the `.proj` file. This file has three
  columns, time, flow, and speed. At the given time instants, the
  corresponding flow and speed values are prescribed. In between, a
  linear interpolation takes place. Outside, a constant extrapolation
  applies. Depending on the actual flow value, new vehicles are drawn
  out of the population given in the `.heterog` file and inserted at
  the prescribed speed. If the prescribed inflow is too high (e.g., if
  a jam wave arrives at the upstream boundary), the flow is reduced
  accordingly and the extraneous vehicles are diverted into an inflow
  buffer to 
  be released once the situation allows for it

* `.BCdown` Specification of the (macroscopic) downstream boundary
  conditions if  `choice_BCdown=0` is set in the `.proj` file. The
  structure is the same as for the `.BCup` specification. Prescribing
  outflows below the local traffic demand, i.e., the local flow immediately
  upstream of the downstream
  boundary, will introduce  congestions.


### Bottlenecks

There are flow-conserving bottlenecks without changing the vehicle
number and on-ramp bottlenecks. This simulator does not implement
off-ramps 

* `.alpha_v0` and `.alpha_T` Flow-conserving bottlenecks: Regions of
  locally changed desired speed or desired time gap. Each file
  consists of two columns, the x values and the corresponding
  multiplicators. Interpolation and extrapolation is performed as in
  the `.IC` file. If, for example, the `.alpha_T` file reads

  ```
  %  x(m)    alpha_T 
  %------------------
  0        0.8
  2000     1.
  ```
  then, the time gap at x=1000m is reduced by a factor of
  0.9. *Notice*: This applies to all models and all parameter sets, so
  a characteristic time gap and desired speed has been identified for
  any model, even if not explicitely given by the parameters, such as
  in the OVM/FVDM for the original tanh-based variant. 

* `.rmps` Location of onramp bottlenecks. Given is the center position
  of the merging part of the onramp, and the length of the merging
  section.

  *Notice 1*: `mic` is essentially a single-link single-lane
  simulator, so onramp inflow is modelled just by dropping vehicles
  into the largest gap inside the merging region.

  *Notice 2* The vehicle composition is given by the `.heterog` file

* `.rmp<n>` Inflow of ramp n as a function of time. `.rmp1`
  corresponds to the first data line of `.rmps`, `.rmp2` to the
  second, and so forth. Inter- and extrapolation is performed as in
  the `.BCup` and `.BCdown` files. The speed is controlled by the
  mainroad vehicles and set to the arithmetic mean of that of the
  mainroad vehicles at either side of the largest gap in the merging
  ("vehicle dropping" zone.


### Output control and output files

In addition to the top-level control in the `.proj` file, there are
some specific controls as follows:

* `.detectors` Locations <loc> of the virtual detectors. Each data line will
  produce three output files:

  * `.x<loc>_det: Aggregated virtual detecor data (sampling interval
    controlled by the settings in the `.proj` file) at a location
    given by <loc> (in meters)

  * `.x<loc>_single: Single vehicle passages as it is recorded by a
    (double-loop) detector before the aggregation step
 
  * `.x<loc>_macro:  In
    contrast to the virtual detectors, also the true spatial values
    (such as the density equal to the inverse of the distance of the
    vehicles on either sde of the cross section) are recorded.

* `.t` Time instants where snapshots of all vehicles are taken

* `.floatcars` vehicle indices for which floating-car data are
  recorded. At the beginning, the first vehicle has the index 1 and
  the vehicle indices are incremented with each new vehicle. So, if a
  vehicle index 4000 is prescribed in `.floatcars`, the `.IC` file
  prescribes an empty road, and the `BCup` file prescribes 2000
  vehicles/h, the floating-car trajectory will start after about 2
  hours of simulated time. Each data line will produce an output file
  `.floatcar<n>`. 

* `.dat` Macroscopic output gridded over the complete spatiotemporal
  simulation range. Activated if `choice_output`=0 or 2 in the `.proj`
  file, and controlled by the `dxout_3D` and `dtout_3D` entries of `.proj`.

* `.micdat` Microscopic output for every dnout_3D'th vehicle
  Activated if `choice_output`=1 or 2 in the `.proj` file and
  controlled by the entry `dnout_3D=<integer>` of `.proj`.

* `.fund<MOD><n>` fundamental diagram produced for the corresponding
  `.<MOD><n>` model specification. In case of "three-phase models"
  with a 2d indifference zone instead of a fundamental diagram, this
  file contains a sample realisation of the indifference zone when
  scanning over the densities.


### Fuel consumption module

* `.engineData<n>`, `.carData<n>` and `.fuel*`  Input and output data
  of the physics-based modal fuel consumption module 



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
