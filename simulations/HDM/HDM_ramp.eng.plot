gnuplot HDM_ramp.eng.gnu
for ext in . .harm. .real. .funddias.; do
  echo "making HDM_ramp${ext}eng.eps"
  fig2dev -L eps HDM_ramp${ext}eng.fig HDM_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT HDM_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT HDM_ramp.eng.eps &
#gv --orientation=PORTRAIT HDM_ramp.real.eng.eps &
gv --orientation=PORTRAIT HDM_ramp.funddias.eng.eps &

