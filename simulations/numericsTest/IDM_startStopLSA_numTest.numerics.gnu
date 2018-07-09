
set style line 1 lt 1 lw 7 pt 7 ps 1.5 lc rgb "#000000"
set style line 2 lt 7 lw 3 pt 5 ps 1.5 lc rgb "#dd0000"
set style line 3 lt 1 lw 7 pt 9 ps 1.5 lc rgb "#ff8800"
set style line 4 lt 3 lw 3 pt 11 ps 1.5 lc rgb "#00aa44"
set style line 5 lt 5 lw 7 pt 13 ps 1.5 lc rgb "#220099"

min(x,y)=(x<y) ? x : y

set term post eps enhanced color dash "Helvetica" 24
set nogrid

set size 1,1
set key left

#########################################################
set out "./IDM_startStopLSA_numTest.numerics_v_dt.eps"
print "plotting ./IDM_startStopLSA_numTest.numerics_v_dt.eps"
#########################################################

set xlabel "ln(update time interval dt [s])"
set ylabel "ln(global error ||vTest-vRef||_1 of car 10 [m/s])"

plot\
     "IDM_startStopLSA_numTest_car10_Method1"\
        u (log($1)):(log($3)) t "Euler"  w l ls 1,\
     "IDM_startStopLSA_numTest_car10_Method2"\
        u (log($1)):(log($3)) t "Ballistic"  w l ls 2,\
     "IDM_startStopLSA_numTest_car10_Method3"\
        u (log($1)):(log($3)) t "Trapezoid"  w l ls 3,\
     "IDM_startStopLSA_numTest_car10_Method4"\
        u (log($1)):(log($3)) t "RK4"  w l ls 4

#########################################################
set out "./IDM_startStopLSA_numTest.numerics_x_dt.eps"
print "plotting ./IDM_startStopLSA_numTest.numerics_x_dt.eps"
#########################################################

set ylabel "ln(global error ||xTest-xRef||_1 of car 10 [m])"

plot\
     "IDM_startStopLSA_numTest_car10_Method1"\
        u (log($1)):(log($2)) t "Euler"  w l ls 1,\
     "IDM_startStopLSA_numTest_car10_Method2"\
        u (log($1)):(log($2)) t "Ballistic"  w l ls 2,\
     "IDM_startStopLSA_numTest_car10_Method3"\
        u (log($1)):(log($2)) t "Trapezoid"  w l ls 3,\
     "IDM_startStopLSA_numTest_car10_Method4"\
        u (log($1)):(log($2)) t "RK4"  w l ls 4

#########################################################
set out "./IDM_startStopLSA_numTest.numerics_v_cost.eps"
print "plotting ./IDM_startStopLSA_numTest.numerics_v_cost.eps"
#########################################################

set xlabel "ln(numerical cost=number of calculations of acc[au])"
set ylabel "ln(global error ||vTest-vRef||_1 of car 10 [m/s])"
set key right

plot\
     "IDM_startStopLSA_numTest_car10_Method1"\
        u (log(1./$1)):(log($3)) t "Euler"  w l ls 1,\
     "IDM_startStopLSA_numTest_car10_Method2"\
        u (log(1./$1)):(log($3)) t "Ballistic"  w l ls 2,\
     "IDM_startStopLSA_numTest_car10_Method3"\
        u (log(2./$1)):(log($3)) t "Trapezoid"  w l ls 3,\
     "IDM_startStopLSA_numTest_car10_Method4"\
        u (log(4./$1)):(log($3)) t "RK4"  w l ls 4
set xlabel "ln(numerical cost=number of calculations of acc[au])"

#########################################################
set out "./IDM_startStopLSA_numTest.numerics_x_cost.eps"
print "plotting ./IDM_startStopLSA_numTest.numerics_x_cost.eps"
#########################################################

plot\
     "IDM_startStopLSA_numTest_car10_Method1"\
        u (log(1./$1)):(log($2)) t "Euler"  w l ls 1,\
     "IDM_startStopLSA_numTest_car10_Method2"\
        u (log(1./$1)):(log($2)) t "Ballistic"  w l ls 2,\
     "IDM_startStopLSA_numTest_car10_Method3"\
        u (log(2./$1)):(log($2)) t "Trapezoid"  w l ls 3,\
     "IDM_startStopLSA_numTest_car10_Method4"\
        u (log(4./$1)):(log($2)) t "RK4"  w l ls 4


quit
