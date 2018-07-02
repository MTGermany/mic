# (pointstyle (s.u.):p 5 = closed square, p 9 = closed triangle)

set style line 99 lt 1 lw 1 linecolor rgb "#000000" # beliebige Farben:Schwarz

set style line 1 lt 7 lw 1 pt 5 ps 1.5 lc  rgb "#000000"
set style line 9 lt 7 lw 1 pt 5 ps 1.5 lc  rgb "#0000AA"
set style line 11 lt 7 lw 3 pt 5 ps 1.5 lc  rgb "#000000" #7=schwarz 
set style line 2 lt 3 lw 2 pt 9 ps 2 #3=blau
set style line 12 lt 3 lw 7 pt 9 ps 1.5 #3=blau, Dreieck Spitze up
set style line 22 lt palette lw 10 pt 2 ps 1.5 #Palette
set style line 3 lt 5 lw 1 pt 5 ps 1.5 #5=tuerkisgruen
set style line 4 lt 2 lw 1 pt 7 ps 1.5 #2=gruen,bullet
set style line 5 lt 6 lw 1 pt 9 ps 1.5 #6=gelb 
set style line 6 lt 8 lw 1 pt 11 ps 1.5 #8 orange, dreieck upside down
set style line 7 lt 4 lw 1 pt 5 ps 1.5 #1=rot
set style line 17 lt 4 lw 3 pt 5 ps 1.5 #4=lila
set style line 8 lt 4 lw 1 pt 5 ps 1.5 #4=lila
set style line 18 lt 4 lw 10 pt 5 ps 1.5 #4=lila


############### 3d ###########

#Gnuplot Bug: Schaltet nicht von Palette zurueck, deshalb -1=Black

set palette defined ( -1 "black", 0 "red", 20 "orange", 40 "yellow",\
      60 "green", 80 "blue",  100 "#dd00ff" ) 
set cbrange [0:140]
set pm3d; set pm3d map
#set contour surface
set cntrparam bspline 
set cntrparam levels 15
unset clabel  # dann lauter gleiche Kontourlinien; 
                     # Farbe und Typ mit "w l ls" beim splot-Kommando


#set view 20,63
#set hidden3d
set ticslevel 0; set nogrid



##################################
set out "./IIDM_ramp_v3d.eng.eps"
print "plotting ./IIDM_ramp_v3d.eng.eps"
##################################

set nokey

set size 1.3,1.2
#set label "(a)" at screen 0.85,0.75
set label 1 "V (km/h)" at screen 1.1,1.08

xmin=-8  # km
xmax=2
xramp2=15.  # km
tmin=0   
tmax=90 # min
tshift=20.  

set term post eps enhanced color solid "Helvetica" 28
set xlabel "Time (min)"
set xrange [tmin:tmax]
set xtics 30
set xtics 30

set ylabel "Distance from onramp (km)" offset -0.5,0
set yrange [xmin:xmax]

set ztics 50
unset surface

splot "IIDM_ramp.dat" u ($2*60-tshift):($1-xramp2):4 w l ls 99



set nolabel
unset colorbox
unset pm3d

##################################
set out "./IIDM_ramp_fund.eng.eps"
print "plotting ./IIDM_ramp_fund.eng.eps"
##################################

#set label "(b)" at 5,2400

set key at screen 0.97,0.92
set size 1,1

rhomin=0.
rhomax=70.
rhomaxReal=110.
Qmin=0.
Qmax=2700.

set xlabel "Density {/Symbol r} (1/km)"
set xrange [rhomin:rhomax]
set xtics 20

set ylabel "Flow Q (1/h)" offset 1,0
set yrange [Qmin:Qmax]

###############################################################################
avgInterval=60   # sampling interval of detectors (s) 
rhofun(q,v)=(v<=0.00001) ? 0  : 3600/avgInterval*q/v
Qfun(q)    =(q<=0.00001) ? 0 : 3600/avgInterval*q
rhoharmFun(invTbrutto,invvharm)=(invTbrutto<0.00001) ? 0 : 3600*invTbrutto*invvharm
QharmFun(invTbrutto)=(invTbrutto<0.00001) ? 0 : 3600*invTbrutto
vharmFun(invvharm)=(invvharm<0.00001) ? 0 : 1./invvharm
###############################################################################


plot\
 "IIDM_ramp.x8000_det" u (rhofun($2, $3)):(Qfun($2)) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (rhofun($2, $3)):(Qfun($2)) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (rhofun($2, $3)):(Qfun($2)) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (rhofun($2, $3)):(Qfun($2)) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_fundDouble.eng.eps"
print "plotting ./IIDM_ramp_fundDouble.eng.eps"
##################################
set xrange [rhomin:rhomaxReal]
plot\
 "IIDM_ramp.x8000_det" u (rhofun($2, $3)):(Qfun($2)) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (rhofun($2, $3)):(Qfun($2)) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (rhofun($2, $3)):(Qfun($2)) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (rhofun($2, $3)):(Qfun($2)) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_fundHarm.eng.eps"
print "plotting ./IIDM_ramp_fundHarm.eng.eps"
##################################
set xrange [rhomin:rhomaxReal]

plot\
 "IIDM_ramp.x8000_det" u (rhoharmFun($5, $4)):(QharmFun($5)) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (rhoharmFun($5, $4)):(QharmFun($5)) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (rhoharmFun($5, $4)):(QharmFun($5)) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (rhoharmFun($5, $4)):(QharmFun($5)) t "-1 km" w p ls 1


##################################
set out "./IIDM_ramp_fundHarm1.eng.eps"
print "plotting ./IIDM_ramp_fundHarm1.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (rhoharmFun($5, $4)):(QharmFun($5)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11

##################################
set out "./IIDM_ramp_fundvHarm1.eng.eps"
print "plotting ./IIDM_ramp_fundvHarm1.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (rhoharmFun($5, $4)):(Qfun($2)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11



####################################################
# Test various forms of macroscopic flow-density defs
####################################################
# Attention! Stipdonk denotes as "harmonic" averaged Q the usuald def
# Q=n/Deltat=1/ (1/n*Deltat)=1/ (1/n*sumi Deltat_i)=1/( 1/n*sumi 1/q_i)=qH
# and does not introduce "my" definition
# QH=1/n sumi(1/Deltat_i)=1/n sumi q_i=<q>

################################## conventionally obtained flow-density data
set out "./IIDM_ramp_fund_Q_vs_QdivV.eng.eps"
print "plotting ./IIDM_ramp_fund_Q_vs_QdivV.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (rhofun($2, $3)):(Qfun($2)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11


################################## ... using harmonic speed
set out "./IIDM_ramp_fund_Q_vs_QdivVH.eng.eps"
print "plotting ./IIDM_ramp_fund_Q_vs_QdivVH.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (Qfun($2)/vharmFun($4)):(Qfun($2)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11

################################## ... using harmonic flow AND harmonic speed
set out "./IIDM_ramp_fund_QH_vs_QHdivVH.eng.eps"
print "plotting ./IIDM_ramp_fund_QH_vs_QHdivVH.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (QharmFun($5)/vharmFun($4)):(QharmFun($5)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11

################################## ... using harmonic flow AND arithmetic speed
set out "./IIDM_ramp_fund_QH_vs_QHdivV.eng.eps"
print "plotting ./IIDM_ramp_fund_QH_vs_QHdivV.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (QharmFun($5)/$3):(QharmFun($5)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11

#... using harmonic speed but harmonic flow ONLY for the density definition!
################################## 
set out "./IIDM_ramp_fund_Q_vs_QHdivVH.eng.eps"
print "plotting ./IIDM_ramp_fund_Q_vs_QHdivVH.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_det" u (QharmFun($5)/vharmFun($4)):(Qfun($2)) t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11



set nolabel


############### funddia with real Density ###########


##################################
set out "./IIDM_ramp_fundTrue.eng.eps"
print "plotting ./IIDM_ramp_fundTrue.eng.eps"
##################################
set xlabel "Density {/Symbol r} (1/km)"
set xrange [0:rhomaxReal]
set xtics 20

set ylabel "Q (1/h)"
set yrange [0:Qmax]

set key at screen 0.97,0.92
set yrange [0:Qmax]
set size 1,1

plot\
 "IIDM_ramp.x8000_macro" u 3:5 t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_macro" u 3:5 t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_macro" u 3:5 t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_macro" u 3:5 t "-1 km" w p ls 1,\
 "IIDM_ramp.fundVW1" u ($1):($4) t "IDM Model" w l ls 11
##################################
set out "./IIDM_ramp_fundTrue1.eng.eps"
print "plotting ./IIDM_ramp_fundTrue1.eng.eps"
##################################
plot\
 "IIDM_ramp.x12000_macro" u 3:5 t "-3 km" w p ls 9,\
  "IIDM_ramp.fundVW1" u ($1):($4) t "" w l ls 11

##################################
set out "./IIDM_ramp_QV.eng.eps"
print "plotting ./IIDM_ramp_QV.eng.eps"
##################################

set key at screen 0.48,0.75

Vmax=125.
set xlabel "Flow Q (1/h)"
set xrange [0:Qmax]
set xtics 500
set ylabel "Speed V (km/h)" offset 1,0
set yrange [0:Vmax]
plot\
 "IIDM_ramp.x8000_det" u (Qfun($2)):($3) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (Qfun($2)):($3) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (Qfun($2)):($3) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (Qfun($2)):($3) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_QVharm.eng.eps"
print "plotting ./IIDM_ramp_QVharm.eng.eps"
##################################
plot\
 "IIDM_ramp.x8000_det" u (QharmFun($5)):(vharmFun($4)) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (QharmFun($5)):(vharmFun($4)) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (QharmFun($5)):(vharmFun($4)) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (QharmFun($5)):(vharmFun($4)) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_QVTrue.eng.eps"
print "plotting ./IIDM_ramp_QVTrue.eng.eps"
##################################
plot\
 "IIDM_ramp.x8000_macro" u 5:4 t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_macro" u 5:4 t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_macro" u 5:4 t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_macro" u 5:4 t "-1 km" w p ls 1,\
 "IIDM_ramp.fundVW1" u ($4):($3) t "IDM Model" w l ls 11


##################################
set out "./IIDM_ramp_rhoV.eng.eps"
print "plotting ./IIDM_ramp_rhoV.eng.eps"
##################################

set key at screen 0.97,0.92

Vmax=125.
set xlabel "Density {/Symbol r} (1/km)"
set xrange [0:rhomax]
set xtics 20

set ylabel "Speed V (km/h)" offset 1,0
set yrange [0:Vmax]
plot\
 "IIDM_ramp.x8000_det" u (rhofun($2, $3)):($3) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (rhofun($2, $3)):($3) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (rhofun($2, $3)):($3) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (rhofun($2, $3)):($3) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_rhoVDouble.eng.eps"
print "plotting ./IIDM_ramp_rhoVDouble.eng.eps"
##################################
set xrange [0:rhomaxReal]
plot\
 "IIDM_ramp.x8000_det" u (rhofun($2, $3)):($3) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (rhofun($2, $3)):($3) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (rhofun($2, $3)):($3) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (rhofun($2, $3)):($3) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_rhoVharm.eng.eps"
print "plotting ./IIDM_ramp_rhoVharm.eng.eps"
##################################
set xrange [0:rhomaxReal]
plot\
 "IIDM_ramp.x8000_det" u (rhoharmFun($5,$4)):(vharmFun($4)) t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_det" u (rhoharmFun($5,$4)):(vharmFun($4)) t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_det" u (rhoharmFun($5,$4)):(vharmFun($4)) t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_det" u (rhoharmFun($5,$4)):(vharmFun($4)) t "-1 km" w p ls 1

##################################
set out "./IIDM_ramp_rhoVTrue.eng.eps"
print "plotting ./IIDM_ramp_rhoVTrue.eng.eps"
##################################
plot\
 "IIDM_ramp.x8000_macro" u 3:4 t "-7 km" w p ls 6,\
 "IIDM_ramp.x10000_macro" u 3:4 t "-5 km" w p ls 4,\
 "IIDM_ramp.x12000_macro" u 3:4 t "-3 km" w p ls 2,\
 "IIDM_ramp.x14000_macro" u 3:4 t "-1 km" w p ls 1,\
 "IIDM_ramp.fundVW1" u ($1):($3) t "IDM Model" w l ls 11



###############################################
set out "./IIDM_ramp_v.eng.eps"
print "plotting ./IIDM_ramp_v.eng.eps"
###############################################

set size 1.2,1
set nokey
set param
set nogrid

tmin=0
tmax=90
xmin=-5.5
xmax=1.7

xD0=1
xD1=0
xD2=-1
xD3=-2
xD4=-3
xD5=-4
xD6=-5

vAvg_kmh=40.
vFun(t,x,v_kmh)=x+0.009*(v_kmh-vAvg_kmh)

set xlabel "t (min)"
set xrange [tmin:tmax]
set xtics 30


set ylabel "x (km)" offset -1,0
set yrange [xmin:xmax]
set ytics 2

unset surface; set surface; set pm3d; set pm3d map

# etliche gnuplot Bugs mit Verwendung der richtigen Linien

splot[t=tmin:tmax] \
 "IIDM_ramp.x16000_det" u  ($1-tshift): (vFun($1, xD0, $3)-0.2):($3) w l ls 22,\
 "IIDM_ramp.x15000_det" u  ($1-tshift): (vFun($1, xD1, $3)):($3) w l ls 22,\
 "IIDM_ramp.x14000_det" u  ($1-tshift): (vFun($1, xD2, $3)):($3) w l ls 22,\
 "IIDM_ramp.x13000_det" u  ($1-tshift): (vFun($1, xD3, $3)):($3) w l ls 22,\
 "IIDM_ramp.x12000_det" u  ($1-tshift): (vFun($1, xD4, $3)):($3) w l ls 22,\
 "IIDM_ramp.x11000_det" u  ($1-tshift): (vFun($1, xD5, $3)):($3) w l ls 22,\
 "IIDM_ramp.x10000_det"   u  ($1-tshift): (vFun($1, xD6, $3)):($3) w l ls 22,\
  t, xD0 ,-1 w l ls 99,\
  t, xD1 ,-1 w l ls 11,\
  t, xD2 ,0 w l ls 11,\
  t, xD3 ,0 w l ls 11,\
  t, xD4 ,0 w l ls 11,\
  t, xD5 ,0 w l ls 11,\
  t, xD6 ,0 w l ls 11,\
  t, xD1-0.01 ,0 w l ls 11,\
  t, xD1+0.01 ,0 w l ls 11,\
  t, xD2-0.01 ,0 w l ls 11,\
  t, xD2+0.01 ,0 w l ls 11


unset pm3d


###############################################
set out "./IIDM_ramp_Q.eng.eps"
print "plotting ./IIDM_ramp_Q.eng.eps"
###############################################

set size 1.2,1
set nokey
set param
set nogrid

# Achtung: Def  Qfun(q) (kleines f) weiter oben

nAvg_invmin=20.
QFun(t,x,n_invmin)=x+0.009*(n_invmin-nAvg_invmin) 


plot[t=tmin:tmax] \
 "IIDM_ramp.x16000_det" u  ($1-tshift): (QFun($1, xD0, $3)-0.2) w l ls 12,  t, xD0 w l ls 1,\
 "IIDM_ramp.x15000_det" u  ($1-tshift): (QFun($1, xD1, $3)) w l ls 12,  t, xD1 w l ls 1,\
 "IIDM_ramp.x14000_det" u  ($1-tshift): (QFun($1, xD2, $3)) w l ls 12,  t, xD2 w l ls 1,\
 "IIDM_ramp.x13000_det" u  ($1-tshift): (QFun($1, xD3, $3)) w l ls 12,  t, xD3 w l ls 1,\
 "IIDM_ramp.x12000_det" u  ($1-tshift): (QFun($1, xD4, $3)) w l ls 12,  t, xD4 w l ls 1,\
 "IIDM_ramp.x11000_det" u  ($1-tshift): (QFun($1, xD5, $3)) w l ls 12,  t, xD5 w l ls 1,\
 "IIDM_ramp.x10000_det"   u  ($1-tshift): (QFun($1, xD6, $3)) w l ls 12,  t, xD6 w l ls 1

