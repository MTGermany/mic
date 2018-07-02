
if gnuplot Gipps_startStopLSA.eng.gnu;
  then echo "gnuplotting OK";
  else echo "gnuplotting failed"; exit -1;
fi
tex2ps Gipps_startStopLSA.eng
#gv --orientation=PORTRAIT Gipps_startStopLSA.s.eng.eps &
#gv --orientation=PORTRAIT Gipps_startStopLSA.v.eng.eps &
#gv --orientation=PORTRAIT Gipps_startStopLSA.a.eng.eps &

