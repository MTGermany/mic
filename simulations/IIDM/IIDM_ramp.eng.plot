gnuplot IIDM_ramp.eng.gnu
for ext in . .harm. .real. .funddias.; do
  echo "making IIDM_ramp${ext}eng.eps"
  fig2dev -L eps IIDM_ramp${ext}eng.fig IIDM_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT IIDM_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT IIDM_ramp.eng.eps &
#gv --orientation=PORTRAIT IIDM_ramp.real.eng.eps &
gv --orientation=PORTRAIT IIDM_ramp.funddias.eng.eps &

