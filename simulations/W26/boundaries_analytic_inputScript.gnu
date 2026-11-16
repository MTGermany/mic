
##########################################################
# boundaries from
# ~/versionedProjects/trafficflowdynamics/tex/figsMicro/analytics_W26.gnu
##########################################################

# !!! adapt model parameters !!!

v0=20.	       # desired free-flow speed in free regime (RF)
s0=3.          # minimum gap  [m]
T1=0.9         # minimum time gap [s] for following
T2=1.8	       # constant gap range [m] of following region (RCF)
dv0=2	       # half-width of rel speed [m/s] in RCF at lower gap limit
tau_a=1000.    # speed relaxation time [s]
a0=2.	       # maximum acceleration
b=2.	       # CHANGED MEANING: comfortable deceleration [m/s^2]
bmax=8.	       # max deceleration[m/s^2] (no real parameter, just cutoff)
indiff=1       # indifference level in CF (0=none, 1=full)
use_vl=0       # !! (1=true, 0=false)


imag={0,1}     # imaginary unit just to stop plotting outside of region bounds
eps=1e-6       # just a small number

##############################################################
# General functions
##############################################################

max(x,y)    = (x>y) ? x : y
min(x,y)    = (x>y) ? y : x
theta(x)=(x>=0) ? 1 : 0

unset grid





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
aOP(s,v,vl)=min(aFF(s,v,vl), -(v-vl)/tau_a + (s-sCF1(v,vl))/tau_a**2)


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
aCR(s,v,vl)=-min(bmax, max(b, (0.5*v*(v-vl)/max(eps,s-s0))**2/b)*theta(v-vl))





# Final W-26 acceleration

accW26(s,v,vl)=\
 (RFF(s,v,vl)) ? aFF(s,v,vl) :\
 (RCF(s,v,vl)) ? aCF(s,v,vl) :\
 (ROP(s,v,vl)) ? aOP(s,v,vl) :\
 (RCL(s,v,vl)) ? aCL(s,v,vl) : aCR(s,v,vl)

