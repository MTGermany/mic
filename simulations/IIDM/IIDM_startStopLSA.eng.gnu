
set style line 1 lt 1 lw 7 pt 7 ps 1.5 lc rgb "#000000"
set style line 2 lt 7 lw 3 pt 5 ps 1.5 lc rgb "#dd0000"
set style line 3 lt 1 lw 7 pt 9 ps 1.5 lc rgb "#ff8800"
set style line 4 lt 3 lw 3 pt 11 ps 1.5 lc rgb "#00aa44"
set style line 5 lt 5 lw 7 pt 13 ps 1.5 lc rgb "#220099"

min(x,y)=(x<y) ? x : y

set term post eps enhanced color dash "Helvetica" 24
set nogrid

tmin=0
tmax=158
tmaxdata=101.  # Um bei Acceleration Kurve zu stoppen

set xrange [tmin:tmax]
set xtics 30

#########################################################
set size 1,0.6
set xlabel "Time [s]"
set key center right


#########################################################
set out "./IIDM_startStopLSA.s.eng.eps"
set ylabel "Gap [s]"
set yrange [0:35]
set ytics 10
plot\
     "IIDM_startStopLSA.car1"  u 1:5 t "Veh 1"  w l ls 1,\
     "IIDM_startStopLSA.car5"  u 1:5 t "Veh 5"  w l ls 2,\
     "IIDM_startStopLSA.car10"  u 1:5 t "Veh 10"  w l ls 3,\
     "IIDM_startStopLSA.car15" u 1:5 t "Veh 15" w l ls 4,\
     "IIDM_startStopLSA.car20" u 1:5 t "Veh 20" w l ls 5

#########################################################
set out "./IIDM_startStopLSA.v.eng.eps"
set ylabel "Speed [km/h]
set yrange [0:55]
plot\
     "IIDM_startStopLSA.car1"  u 1:(3.6*$3) t "Veh 1"  w l ls 1,\
     "IIDM_startStopLSA.car5"  u 1:(3.6*$3) t "Veh 5"  w l ls 2,\
     "IIDM_startStopLSA.car10"  u 1:(3.6*$3) t "Veh 10"  w l ls 3,\
     "IIDM_startStopLSA.car15"  u 1:(3.6*$3) t "Veh 15"  w l ls 4,\
     "IIDM_startStopLSA.car20" u 1:(3.6*$3) t "Veh 20" w l ls 5

#########################################################
set out "./IIDM_startStopLSA.a.eng.eps"
set ylabel "Acceleration [m/s^2]"
set auto y
set ytics 1
plot\
     "IIDM_startStopLSA.car1"  u (min($1,tmaxdata)):4 t "Veh 1"  w l ls 1,\
     "IIDM_startStopLSA.car5"  u (min($1,tmaxdata)):4 t "Veh 5"  w l ls 2,\
     "IIDM_startStopLSA.car10"  u (min($1,tmaxdata)):4 t "Veh 10"  w l ls 3,\
     "IIDM_startStopLSA.car15"  u (min($1,tmaxdata)):4 t "Veh 15"  w l ls 4,\
     "IIDM_startStopLSA.car20" u (min($1,tmaxdata)):4 t "Veh 20" w l ls 5

#########################################################
set term post eps enhanced color solid "Helvetica" 20
quit

set yrange [-100:140]
plot[t=0:1]\
      tmin+t*(tmax-tmin), 0 w l 0,\
     "IIDM_startStopLSA.car1"  u (min($1,tmaxdata)):5         t "car1: s(m)"    w p ls 2,\
     "IIDM_startStopLSA.car1"  u (min($1,tmaxdata)):(10*$4)   t "a (0.1*m/s^2)" w l ls 3,\
     "IIDM_startStopLSA.car1"  u (min($1,tmaxdata)):(3.6*$3) t "v (10 km/h)"   w l ls 6

quit
####################################################################

