gnuplot PTmodel_ramp.eng.gnu
for ext in . .harm. .real.; do
  echo "making PTmodel_ramp${ext}eng.eps"
  fig2dev -L eps PTmodel_ramp${ext}eng.fig PTmodel_ramp${ext}eng.eps 
done
gv --orientation=PORTRAIT PTmodel_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT PTmodel_ramp_fundHarm.eng.eps &
#gv --orientation=PORTRAIT PTmodel_ramp_v.eng.eps &
#gv --orientation=PORTRAIT PTmodel_ramp_fundTrue.eng.eps &
#gv --orientation=PORTRAIT PTmodel_ramp_v3d.eng.eps &
