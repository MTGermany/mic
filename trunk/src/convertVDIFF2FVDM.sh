# source code:

cdmic
make clean
mmv "VDIFF.*" "FVDM.#1"
perl -i -p -e 's/VDIFF/FVDM/g' *
make mic

# simulations:

cdmicsim
baseDir=$PWD

for dir in `find . -type "d"`; do
#for dir in ./calibrProfiles; do
  cd $dir
  echo "going to $dir..";
  mmv "*VDIFF*" "#1FVDM#2";
  for f in `find . -name "*FVDM*"`; do perl -i -p -e 's/VDIFF/FVDM/g' $f; done
  cd $baseDir;
done

# other cleaning:

perl -i -p -e 's/7\=FVDM\,7\=FVDM/7=FVDM\,8=Kerner/g' */*.het*

