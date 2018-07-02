
set style line 1 lt 1 lw 7 pt 7 ps 1.5 lc rgb "#000000"
set style line 2 lt 7 lw 3 pt 5 ps 1.5 lc rgb "#dd0000"
set style line 3 lt 1 lw 7 pt 9 ps 1.5 lc rgb "#ff8800"
set style line 4 lt 3 lw 3 pt 11 ps 1.5 lc rgb "#00aa44"
set style line 5 lt 5 lw 7 pt 13 ps 1.5 lc rgb "#220099"

min(x,y)=(x<y) ? x : y

set term post eps enhanced color dash "Helvetica" 20
set nogrid

tmin=0
tmax=158
tmaxdata=101.  # Um bei Beschl. Kurve zu stoppen

set xrange [tmin:tmax]
set xtics 30

#########################################################
set size 1,0.5
set xlabel ""
#set size 1,0.6
#set xlabel "Time [s]"
set key center right


#########################################################
set out "./PTmodel_startStopLSA.s.eng.eps"
print "plotting ./PTmodel_startStopLSA.s.eng.eps"
set ylabel "Gap [s]"
set yrange [0:35]
set ytics 10
plot\
     "PTmodel_startStopLSA.car1"  u 1:5 t "Kfz 1"  w l ls 1,\
     "PTmodel_startStopLSA.car5"  u 1:5 t "Kfz 5"  w l ls 2,\
     "PTmodel_startStopLSA.car10"  u 1:5 t "Kfz 10"  w l ls 3,\
     "PTmodel_startStopLSA.car15" u 1:5 t "Kfz 15" w l ls 4,\
     "PTmodel_startStopLSA.car20" u 1:5 t "Kfz 20" w l ls 5

#########################################################
set out "./PTmodel_startStopLSA.v.eng.eps"
print "plotting ./PTmodel_startStopLSA.v.eng.eps"
set ylabel "V [km/h]
set yrange [0:55]
plot\
     "PTmodel_startStopLSA.car1"  u 1:(3.6*$3) t "Kfz 1"  w l ls 1,\
     "PTmodel_startStopLSA.car5"  u 1:(3.6*$3) t "Kfz 5"  w l ls 2,\
     "PTmodel_startStopLSA.car10"  u 1:(3.6*$3) t "Kfz 10"  w l ls 3,\
     "PTmodel_startStopLSA.car15"  u 1:(3.6*$3) t "Kfz 15"  w l ls 4,\
     "PTmodel_startStopLSA.car20" u 1:(3.6*$3) t "Kfz 20" w l ls 5

#########################################################
set size 1,0.55
set out "./PTmodel_startStopLSA.a.eng.eps"
print "plotting ./PTmodel_startStopLSA.a.eng.eps"
set xlabel "Time [s]"
set ylabel "Accel. [m/s^2]"
set auto y
set ytics 1
plot\
     "PTmodel_startStopLSA.car1"  u (min($1,tmaxdata)):4 t "Kfz 1"  w l ls 1,\
     "PTmodel_startStopLSA.car5"  u (min($1,tmaxdata)):4 t "Kfz 5"  w l ls 2,\
     "PTmodel_startStopLSA.car10"  u (min($1,tmaxdata)):4 t "Kfz 10"  w l ls 3,\
     "PTmodel_startStopLSA.car15"  u (min($1,tmaxdata)):4 t "Kfz 15"  w l ls 4,\
     "PTmodel_startStopLSA.car20" u (min($1,tmaxdata)):4 t "Kfz 20" w l ls 5

#########################################################
quit
