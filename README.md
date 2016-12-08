# mic

a small versatile effectively single-lane microscopic traffic flow
simulator for model tests. Merges are modelled in a very simple way
just by adding 
vehicles into the largest gap inside the merging region with the
inflow specified in a .rmp<n> file (see below)

## Compiling

This simulator uses c++
on a linux system, just enter "make mic" in the subdirectory /trunc/src/

## Running the Simulation


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
