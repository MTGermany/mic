gnuplot Gipps_ramp.eng.gnu
for ext in . .harm. .real. .funddias.; do
  echo "making Gipps_ramp${ext}eng.eps"
  fig2dev -L eps Gipps_ramp${ext}eng.fig Gipps_ramp${ext}eng.eps 
done
#gv --orientation=PORTRAIT Gipps_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT Gipps_ramp.eng.eps &
#gv --orientation=PORTRAIT Gipps_ramp.real.eng.eps &
gv --orientation=PORTRAIT Gipps_ramp.funddias.eng.eps &

