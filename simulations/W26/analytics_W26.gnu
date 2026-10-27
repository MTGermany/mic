
##########################################################
# New "Wiedemann 26" model as developed by Venkat Kanagaraj
# and modified by me
##########################################################


unset grid

##############################################################
# General functions
##############################################################

max(x,y)    = (x>y) ? x : y
min(x,y)    = (x>y) ? y : x
theta(x)=(x>=0) ? 1 : 0



# Parameters # meaning [units] 

v0=30.	       # desired free-flow speed in free regime (RF)
s0=3.          # minimum gap  [m]
T1=0.9          # minimum time gap [s] for following
T2=1.8	       # constant gap range [m] of following region (RCF)
dv0=2	       # half-width of rel speed [m/s] in RCF at lower gap limit
tau_a=1000.      # speed relaxation time [s]
a0=2.	       # maximum acceleration
b=2.	       # CHANGED MEANING: comfortable deceleration [m/s^2]
bmax=8.	       # max deceleration[m/s^2] (no real parameter, just cutoff)
indiff=1     # indifference level in CF (0=none, 1=full)
use_vl=0     # !! (1=true, 0=false)


imag={0,1}     # imaginary unit just to stop plotting outside of region bounds
eps=1e-6       # just a small number


#########################################################################
# Thresholds and acceleration functions (s denotes bumper-to-bumper gap)
# * If in actual applications v=arg (use_vl=0=false)
#   =>oblique regions if vl=const
# * If use_vl=1=true => oblique regions if v=const
#########################################################################
use_vl=0  # (1=true, 0=false)
#########################################################################

dsCF(v,vl)=(use_vl) ? max(s0,(T2-T1)*vl) : max(s0,(T2-T1)*v)  # !!! max(s0,.)
sCF1(v,vl)=(use_vl) ? s0+T1*vl: s0+T1*v

sCF2(v,vl)=sCF1(v,vl)+dsCF(v,vl)
dvCF(s,v,vl)=dv0*(1.+max((s-sCF1(v,vl))/dsCF(v,vl),0.)) # D(s) as modified by me

# (1) Free-flow regime RFF

RFF(s,v,vl)=(s>=sCF2(v,vl)) && (abs(v-vl)<dvCF(s,v,vl))
RFFfun(s,v,vl)=(RFF(s,v,vl)) ? a0 : imag;      # for W26_regions.eps
aFF(s,v,vl)=a0*(1-v/v0)	       	    	       # for W26_acc.eps

# (2) Following regime RCF (sCFxcropped for W26_boundaries.eps)

sCF1cropped(v,vl)=(abs(v-vl)<=dvCF(sCF1(v,vl),v,vl)) ? sCF1(v,vl) : imag;
sCF2cropped(v,vl)=(abs(v-vl)<=dvCF(sCF2(v,vl),v,vl)) ? sCF2(v,vl) : imag;

RCF(s,v,vl)=(s>=sCF1(v,vl)) && (s<=sCF2(v,vl)) && (abs(v-vl)<=dvCF(s,v,vl))
RCFfun(s,v,vl)=(RCF(s,v,vl)) ? 0 : imag;
aCF(s,v,vl)= min(aFF(s,v,vl), (1-indiff)*((s-sCF1(v,vl))/tau_a**2-(v-vl)/tau_a))




# (3) Opening regime ROP
# (no new thresholds because defined by other regimes)

sOPcropped(v,vl)=(v-vl<-dvCF(sCF1(v,vl),v,vl)) ? sCF1(v,vl)-(v-vl+dvCF(sCF1(v,vl),v,vl))*dsCF(v,vl)/dv0 : imag

ROP(s,v,vl)=(v-vl<-dvCF(s,v,vl))
ROPfun(s,v,vl)=(ROP(s,v,vl)) ? a0/2. : imag;
#aOP(s,v,vl)=min(aFF(s,v,vl), -(v-vl)/tau_a + (s-sCF1(v,vl))/tau_a**2)
aOP(s,v,vl)=aFF(s,v,vl)


# (4) Closing regime RCL
# (no new thresholds because defined by other regimes)

sCLcropped(v,vl)=(v-vl>dvCF(sCF1(v,vl),v,vl)) ? sCF1(v,vl)+(v-vl-dvCF(sCF1(v,vl),v,vl))*dsCF(v,vl)/dv0 : imag

RCL(s,v,vl)=(s>=sCF1(v,vl)) && ((v-vl)>dvCF(s,v,vl))
RCLfun(s,v,vl)=(RCL(s,v,vl)) ? -b : imag;
#aCL(s,v,vl)= -min(bmax, max((v-vl)/tau_a, 0.5*(v-vl)**2/max(eps,s-s0)))
aCL(s,v,vl)= -min(bmax, max((v-vl)/tau_a, (0.5*v*(v-vl)/max(eps,s-s0))**2/b))



# (5) critical regime RCR !! never <-b CHANGE!

sCRcropped(v,vl)=(v-vl>-dvCF(sCF1(v,vl),v,vl)) ? sCF1(v,vl) : imag;

RCR(s,v,vl)=(s<sCF1(v,vl)) && (v-vl>-dv0)
RCRfun(s,v,vl)=(RCR(s,v,vl)) ? -bmax : imag;
#aCR(s,v,vl)=-min(bmax, max(b, 0.5* (v-vl)**2 * theta(v-vl)/max(eps,s-s0)))
aCR(s,v,vl)=-min(bmax, max(b, (0.5*v*(v-vl)/max(eps,s-s0))**2/b*theta(v-vl)))





# Final W-26 acceleration

accW26(s,v,vl)=\
 (RFF(s,v,vl)) ? aFF(s,v,vl) :\
 (RCF(s,v,vl)) ? aCF(s,v,vl) :\
 (ROP(s,v,vl)) ? aOP(s,v,vl) :\
 (RCL(s,v,vl)) ? aCL(s,v,vl) : aCR(s,v,vl)

############################################################################
# tests
############################################################################

vlT=20.
vT=24.
sT=25.
print "vl=",vlT," v=",vT," sT=",sT," dv0=",dv0," dsCF(v,vl)=",dsCF(vT,vlT)
print "sCF1(v,vl)=",sCF1(vT,vlT)
print "sCF1(v,vl)+dsCF(v,vl)=",sCF1(vT,vlT)+dsCF(vT,vlT)
print "dvCF(sT,v,vl)=",dvCF(sT,vT,vlT)
print "sOPcropped(v,v)=",sOPcropped(vT,vT)

print "RFF(s,v,vl)=",RFF(sT, vT, vlT)," aFF(s,v,vl)=",aFF(sT, vT, vlT)
print "RCF(s,v,vl)=",RCF(sT, vT, vlT)," aCF(s,v,vl)=",aCF(sT, vT, vlT)
print "RCL(s,v,vl)=",RCL(sT, vT, vlT)," aCL(s,v,vl)=",aCL(sT, vT, vlT)
print "ROP(s,v,vl)=",ROP(sT, vT, vlT)," aOP(s,v,vl)=",aOP(sT, vT, vlT)
print "RCR(s,v,vl)=",RCR(sT, vT, vlT)," aCR(s,v,vl)=",aCR(sT, vT, vlT)
print "accW26(s,v,vl)=",accW26(sT, vT, vlT)

##############################################################
# plot of the region boundaries for v=20 m/s
##############################################################

set term post eps enhanced color solid "Helvetica" 16
set out "W26_boundaries.eps"
set samples 200

print "\nplotting W26_boundaries.eps"

######################################################
vl=20
str_vl=sprintf("v_l=%1.0f m/s", vl)
######################################################

dvmin=max(-vl,-8.) # approaching rate v-v_l as in VISSIM Wiedemann plots
dvmax=10. 

smin=0.
smax=60

set style line 99 lt 1 lw 2 pt 7 ps 1.9 dt 1 lc rgb "#000000" #schwarz,bullets

set style line 1 lt 1 lw 2 pt 7 ps 1.2  lc rgb "#880000" #AX
set style line 2 lt 1 lw 2 pt 2 ps 1.0  lc rgb "#DD0000" #ABX
set style line 3 lt 8 lw 2 pt 4 ps 1.0  lc rgb "#33CC00" #SDX
set style line 4 lt 6 lw 2 pt 4 ps 0.7  lc rgb "#FFBB00" #SDV
set style line 5 lt 1 lw 2 pt 5 ps 0.5  lc rgb "#FF8800" #CLDV, nearly same color
set style line 6 lt 5 lw 2 pt 4 ps 1.0  lc rgb "#0033AA" #OPDV

set xlabel "Approaching rate v-v_l [m/s]"
set xrange [dvmin:dvmax]
set ylabel "Bumper-to-bumper gap [m]"
set yrange [smin:smax]
set label str_vl at screen 0.45,0.92

set key box opaque top left

set samples 200

set label "free" at 0,1.2*(sCF1(vl,vl)+dsCF(vl,vl)) center front  # v->vl for labels
# possibly other free
set label "following" at 0,sCF1(vl,vl)+0.5*dsCF(vl,vl) center front
set label "closing" at dvCF(1.2*(sCF1(vl,vl)+dsCF(vl,vl)),vl,vl),sCF1(vl,vl)+0.5*dsCF(vl,vl) center front
set label "opening" at -dvCF(1.2*(sCF1(vl,vl)+dsCF(vl,vl)),vl,vl),sCF1(vl,vl)+0.5*dsCF(vl,vl) center front
set label "critical" at 0, 0.5*sCF1(vl,vl) center front


plot [v=vl+dvmin:vl+dvmax]\
  v-vl, sCF1cropped(v,vl) t "CF1" w l ls 1,\
  v-vl, sCF2cropped(v,vl) t "CF2" w l ls 2,\
  v-vl, sCLcropped(v,vl) t "CL" w l ls 3,\
  v-vl, sOPcropped(v,vl) t "OP" w l ls 5,\
  v-vl, sCRcropped(v,vl) t "CR" w l ls 8,\
  -dv0,sCF1(vl-dv0,vl)*(v-vl-dvmin)/(dvmax-dvmin) t "" w l ls 8


#############################################################
print "\nRegions:"
#############################################################

set out "W26_regions.eps"
print "plotting W26_regions.eps"
set isosamples 100
set nokey
set palette defined (-bmax "#660044", -b "red",\
     -b/2 "orange", 0 "yellow",\
     0.6*a0 "green", a0 "#008866")

set cbrange [-bmax:a0]  #[min:max] of color-coded  range 
set auto z
set pm3d; set pm3d map                     # color coded 3d surface
set contour surface      # Aktiviert Kontourlinien auf 3D-Flaeche
set cntrparam levels incremental -bmax,0.2,a0
unset clabel   # crucial, otherwise DOS at ls 99
unset surface  # also crucial for avoiding DOS at ls 99


splot [v=vl+dvmin:vl+dvmax][s=smin:smax]\
 v-vl, s, RCFfun(s,v,vl) t "CF region" w l ls 99 lc rgb "#000000" ,\
 v-vl, s, RFFfun(s,v,vl) t "Free region" w l ls 99,\
 v-vl, s, ROPfun(s,v,vl) t "Opening region" w l ls 99,\
 v-vl, s, RCLfun(s,v,vl) t "Closinging region" w l ls 99,\
 v-vl, s, RCRfun(s,v,vl) t "Critical region" w l ls 99


#############################################################
print "\nAccelerations:"
#############################################################

set out "W26_acc.eps"
print "plotting W26_acc.eps"
splot [v=vl+dvmin:vl+dvmax][s=smin:smax]\
 v-vl, s, accW26(s,v,vl) w l ls 99

quit
