gnuplot fuel_IDM_ramp.eng.gnu
#for ext in . .harm. .real. .funddias.; do
for ext in .; do
  echo "making fuel_IDM_ramp${ext}eng.eps"
  fig2dev -L eps fuel_IDM_ramp${ext}eng.fig fuel_IDM_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT fuel_IDM_ramp.harm.eng.eps &
gv --orientation=PORTRAIT fuel_IDM_ramp.eng.eps &
#gv --orientation=PORTRAIT fuel_IDM_ramp.real.eng.eps &
#gv --orientation=PORTRAIT fuel_IDM_ramp.funddias.eng.eps &

