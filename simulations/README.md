## Running the Simulation (copied from the toplevel README.md)

The simulations are organized into `projects`. Each project has a
variable number of input and output files to be explained below. 
you can find available projects by the bash command

```
for f in `find . -name "*.proj"`; do echo "`dirname $f`/`basename $f .proj`"; done
```

* You can run the simulation on this project just by calling mic:
```
mic <projectName>
```

* If you also want some graphics produced with gnuplot and/or xfig, you can also call

```
<projectName>.run
```
However, this can only be done inside the directory containing the project.


* Inside the respective directories, there may also be other gnuplot script files, e.g. `analytics_W26.gnu` in  `simulations/W26/` plotting the boundaries of the different traffic regimes and the acceleration function of the Wiedemann-like W-26 model

* Examples:

+ `mic simulations/IDM/IDM_startStopCity` performs the simulation in the target directory `simulations/IDM/`

+ `cd simulations/IDM/; IDM_startStopCity.run` calls mic and then some gnuplot and/or xfig scripts to produce pdfs or eps files

