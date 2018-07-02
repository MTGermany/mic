gnuplot OVM_ramp.eng.gnu
for ext in . .harm. .real. .funddias.; do
  echo "making OVM_ramp${ext}eng.eps"
  fig2dev -L eps OVM_ramp${ext}eng.fig OVM_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT OVM_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT OVM_ramp.eng.eps &
#gv --orientation=PORTRAIT OVM_ramp.real.eng.eps &
gv --orientation=PORTRAIT OVM_ramp.funddias.eng.eps &

