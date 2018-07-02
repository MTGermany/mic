gnuplot ACC_ramp.eng.gnu
for ext in . .harm. .real. .funddias.; do
  echo "making ACC_ramp${ext}eng.eps"
  fig2dev -L eps ACC_ramp${ext}eng.fig ACC_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT ACC_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT ACC_ramp.eng.eps &
#gv --orientation=PORTRAIT ACC_ramp.real.eng.eps &
gv --orientation=PORTRAIT ACC_ramp.funddias.eng.eps &

