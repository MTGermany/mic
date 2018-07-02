gnuplot KCA_ramp.eng.gnu
gv --orientation=PORTRAIT KCA_ramp_v3d.eng.eps &
gv --orientation=PORTRAIT KCA_ramp_fund.eng.eps
#gv --orientation=PORTRAIT KCA_ramp.eng.eps &
#gv --orientation=PORTRAIT KCA_ramp.harm.eng.eps &
#gv --orientation=PORTRAIT KCA_ramp.real.eng.eps &
#gv --orientation=PORTRAIT KCA_ramp.funddias.eng.eps &
#for ext in . .harm. .real. .funddias.; do
#  echo "making KCA_ramp${ext}eng.eps"
#  fig2dev -L eps KCA_ramp${ext}eng.fig KCA_ramp${ext}eng.eps 
#done

