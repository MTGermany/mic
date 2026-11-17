#mic W26_following
if gnuplot W26_following.gnu; then echo "gnuplot succesful";
else echo "gnuplot failed"; exit -1; fi
tex2pdf W26_following > tmp
echo "created W26_following.sdv.eps and W26_following.pdf"



