gnuplot IDM_ramp.eng.gnu
#for ext in . .harm. .real. .funddias.; do
for ext in .; do
  echo "making IDM_ramp${ext}eng.eps"
  fig2dev -L eps IDM_ramp${ext}eng.fig IDM_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT IDM_ramp.harm.eng.eps &
gv --orientation=PORTRAIT IDM_ramp.eng.eps &
#gv --orientation=PORTRAIT IDM_ramp.real.eng.eps &
#gv --orientation=PORTRAIT IDM_ramp.funddias.eng.eps &

