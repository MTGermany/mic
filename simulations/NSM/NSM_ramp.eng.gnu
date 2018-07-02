# (pointstyle (s.u.):p 5 = closed square, p 9 = closed triangle)

set style line 99 lt 1 lw 1 linecolor rgb "#000000" # beliebige Farben:Schwarz

set style line 1 lt 7 lw 3 pt 5 ps 1.5 lc  rgb "#000000"
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

set palette defined ( 0 "#aa0000", 5 "red", 20 "orange", 40 "yellow",\
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

###############  Skalierungen KCA #############
xscale=7.5
tscale=1.
vscale=xscale/tscale
rhoscale=1./xscale
accscale=xscale/tscale**2
###########################################


##################################
set out "NSM_ramp_v3d.eng.eps"
print "plotting NSM_ramp_v3d.eng.eps"
##################################

set nokey

set size 1.3,1.2
#set label "(a)" at screen 0.85,0.75
set label 1 "V (km/h)" at screen 1.1,1.08

xmin=-8  # km
xmax=2
xramp=15.  # km
tmin=0   
tmax=90 # min
tshift=20.  

set term post eps enhanced color solid "Helvetica" 28
set xlabel "Time (min)"
set xrange [tmin:tmax]
set xtics 30

set ylabel "Distance from onramp (km)" offset -0.5,0
set yrange [xmin:xmax]

set ztics 50
unset surface

splot "NSM_ramp.dat" u ($2*60-tshift):(xscale*$1-xramp):(vscale*$4) w l ls 99
set nolabel
unset colorbox
unset pm3d

##################################
set out "NSM_ramp_fund.eng.eps"
print "plotting NSM_ramp_fund.eng.eps"
##################################

#set label "(b)" at 5,2400

set key at screen 0.97,0.92
set size 1,1

rhomin=0.
rhomax=70.
Qmin=0.
Qmax=2700.

set xlabel "Density {/Symbol r} (1/km)"
set xrange [rhomin:rhomax]
set xtics 20

set ylabel "Flow Q (1/h)" offset 1,0
set yrange [Qmin:Qmax]

###############
avgInterval=60   # sampling interval of detectors (s) 
rhofun(q,v)=(v<=0.00001) ? 0  : 3600/avgInterval*q/v
Qfun(q)    =(q<=0.00001) ? 0 : 3600/avgInterval*q
rhoharmFun(invTbrutto,invvharm)=(invTbrutto<0.00001) ? 0 : 3600*invTbrutto*invvharm
QharmFun(invTbrutto)=(invTbrutto<0.00001) ? 0 : 3600*invTbrutto
vharmFun(invvharm)=(invvharm<0.00001) ? 0 : 1./invvharm
################

plot\
 "NSM_ramp.x1200_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-1 km" w p ls 1

##################################
set out "NSM_ramp_fundDouble.eng.eps"
print "plotting NSM_ramp_fundDouble.eng.eps"
##################################
set xrange [rhomin:2*rhomax]
plot\
 "NSM_ramp.x1200_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (rhofun($2, vscale*$3)):(Qfun($2)) t "-1 km" w p ls 1

##################################
set out "NSM_ramp_fundHarm.eng.eps"
print "plotting NSM_ramp_fundHarm.eng.eps"
##################################
set xrange [rhomin:2*rhomax]

plot\
 "NSM_ramp.x1200_det" u (rhoharmFun($5, $4/vscale)):(QharmFun($5)) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (rhoharmFun($5, $4/vscale)):(QharmFun($5)) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (rhoharmFun($5, $4/vscale)):(QharmFun($5)) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (rhoharmFun($5, $4/vscale)):(QharmFun($5)) t "-1 km" w p ls 1

set nolabel


############### funddia with real Density ###########


set out "NSM_ramp_fundTrue.eng.eps"
print "plotting NSM_ramp_fundTrue.eng.eps"
set xlabel "Density {/Symbol r} (Veh/km)"
set xrange [0:140]
set xtics 20

set ylabel "Q (Veh/h)"
set yrange [0:Qmax]

set key at screen 0.97,0.92
set yrange [0:Qmax]
set size 1,1

plot\
 "NSM_ramp.x1200_macro" u (rhoscale*$3):($5) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_macro" u (rhoscale*$3):($5) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_macro" u (rhoscale*$3):($5) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_macro" u (rhoscale*$3):($5) t "-1 km" w p ls 1,\
 "NSM_ramp.fundKCA1" u (rhoscale*$1):($4) t "IDM" w l ls 11

##################################
set out "NSM_ramp_QV.eng.eps"
print "plotting NSM_ramp_QV.eng.eps"
##################################

set key at screen 0.48,0.75

Vmax=125.
set xlabel "Flow Q (1/h)"
set xrange [0:Qmax]
set xtics 500
set ylabel "Speed V (km/h)" offset 1,0
set yrange [0:Vmax]
plot\
 "NSM_ramp.x1200_det" u (Qfun($2)):(vscale*$3) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (Qfun($2)):(vscale*$3) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (Qfun($2)):(vscale*$3) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (Qfun($2)):(vscale*$3) t "-1 km" w p ls 1,\
 "NSM_ramp.fundKCA1" u ($4/vscale):(vscale*$3) t "IDM" w l ls 11

##################################
set out "NSM_ramp_QVharm.eng.eps"
print "plotting NSM_ramp_QVharm.eng.eps"
##################################
plot\
 "NSM_ramp.x1200_det" u (QharmFun($5)):(vharmFun($4/vscale)) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (QharmFun($5)):(vharmFun($4/vscale)) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (QharmFun($5)):(vharmFun($4/vscale)) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (QharmFun($5)):(vharmFun($4/vscale)) t "-1 km" w p ls 1,\
 "NSM_ramp.fundKCA1" u ($4/vscale):(vscale*$3) t "IDM" w l ls 11


##################################
set out "NSM_ramp_rhoV.eng.eps"
print "plotting NSM_ramp_rhoV.eng.eps"
##################################

set key at screen 0.97,0.92

Vmax=125.
set xlabel "Density {/Symbol r} (Veh/km)"
set xrange [0:60]
set xtics 20

set ylabel "Speed V (km/h)" offset 1,0
set yrange [0:Vmax]
plot\
 "NSM_ramp.x1200_det" u (rhofun($2, vscale*$3)):(vscale*$3) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (rhofun($2, vscale*$3)):(vscale*$3) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (rhofun($2, vscale*$3)):(vscale*$3) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (rhofun($2, vscale*$3)):(vscale*$3) t "-1 km" w p ls 1

##################################
set out "NSM_ramp_rhoVharm.eng.eps"
print "plotting NSM_ramp_rhoVharm.eng.eps"
##################################
set xrange [0:100]
plot\
 "NSM_ramp.x1200_det" u (rhoharmFun($5,$4/vscale)):(vharmFun($4/vscale)) t "-7 km" w p ls 6,\
 "NSM_ramp.x1400_det" u (rhoharmFun($5,$4/vscale)):(vharmFun($4/vscale)) t "-5 km" w p ls 4,\
 "NSM_ramp.x1600_det" u (rhoharmFun($5,$4/vscale)):(vharmFun($4/vscale)) t "-3 km" w p ls 2,\
 "NSM_ramp.x1800_det" u (rhoharmFun($5,$4/vscale)):(vharmFun($4/vscale)) t "-1 km" w p ls 1


###############################################
set out "NSM_ramp_v.eng.eps"
print "plotting NSM_ramp_v.eng.eps"
###############################################

set size 1.2,1
set nokey
set param
set nogrid

tmin=0
tmax=90
xmin=-5.5
xmax=1.7
print "xramp=",xramp
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
 "NSM_ramp.x2000_det" u  ($1-tshift): (vFun($1, xD0, vscale*$3)-0.2):(vscale*$3) w l ls 22,\
 "NSM_ramp.x1900_det" u  ($1-tshift): (vFun($1, xD1, vscale*$3)):(vscale*$3) w l ls 22,\
 "NSM_ramp.x1800_det" u  ($1-tshift): (vFun($1, xD2, vscale*$3)):(vscale*$3) w l ls 22,\
 "NSM_ramp.x1700_det" u  ($1-tshift): (vFun($1, xD3, vscale*$3)):(vscale*$3) w l ls 22,\
 "NSM_ramp.x1600_det" u  ($1-tshift): (vFun($1, xD4, vscale*$3)):(vscale*$3) w l ls 22,\
 "NSM_ramp.x1500_det" u  ($1-tshift): (vFun($1, xD5, vscale*$3)):(vscale*$3) w l ls 22,\
 "NSM_ramp.x1400_det"   u  ($1-tshift): (vFun($1, xD6, vscale*$3)):(vscale*$3) w l ls 22,\
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
set out "NSM_ramp_Q.eng.eps"
print "plotting NSM_ramp_Q.eng.eps"
###############################################

set size 1.2,1
set nokey
set param
set nogrid

# Achtung: Def  Qfun(q) (kleines f) weiter oben

nAvg_invmin=20.
QFun(t,x,n_invmin)=x+0.009*(n_invmin-nAvg_invmin) 


plot[t=tmin:tmax] \
 "NSM_ramp.x2000_det" u  ($1-tshift): (QFun($1, xD0, vscale*$3)-0.2) w l ls 12,  t, xD0 w l ls 1,\
 "NSM_ramp.x1900_det" u  ($1-tshift): (QFun($1, xD1, vscale*$3)) w l ls 12,  t, xD1 w l ls 1,\
 "NSM_ramp.x1800_det" u  ($1-tshift): (QFun($1, xD2, vscale*$3)) w l ls 12,  t, xD2 w l ls 1,\
 "NSM_ramp.x1700_det" u  ($1-tshift): (QFun($1, xD3, vscale*$3)) w l ls 12,  t, xD3 w l ls 1,\
 "NSM_ramp.x1600_det" u  ($1-tshift): (QFun($1, xD4, vscale*$3)) w l ls 12,  t, xD4 w l ls 1,\
 "NSM_ramp.x1500_det" u  ($1-tshift): (QFun($1, xD5, vscale*$3)) w l ls 12,  t, xD5 w l ls 1,\
 "NSM_ramp.x1400_det"   u  ($1-tshift): (QFun($1, xD6, vscale*$3)) w l ls 12,  t, xD6 w l ls 1

