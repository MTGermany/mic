gnuplot FVDM_ramp.eng.gnu
for ext in . .harm. .real.; do
  echo "making FVDM_ramp${ext}eng.eps"
  fig2dev -L eps FVDM_ramp${ext}eng.fig FVDM_ramp${ext}eng.eps 
done
gv --orientation=PORTRAIT FVDM_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT FVDM_ramp_fundHarm.eng.eps &
#gv --orientation=PORTRAIT FVDM_ramp_v.eng.eps &
#gv --orientation=PORTRAIT FVDM_ramp_fundTrue.eng.eps &
#gv --orientation=PORTRAIT FVDM_ramp_v3d.eng.eps &
