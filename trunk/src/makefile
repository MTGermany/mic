#
# makefile of mic project
#


OUTNAME=mic
OBJDIR=.
LIBDIR=~/versionedProjects/lib/trunk
BINDIR=~/bin


OBJECTS=Main.o CyclicBuffer.o ControlledInflow.o Detector.o RoadSection.o ProjectParams.o Heterogen.o Vehicle.o Ramp.o ExternalControl.o Boundary.o FloatingCars.o VehicleDropLift.o Fluct.o Consumption.o TrajectoryLOS.o  MicroModel.o IDM.o VW.o NewModel.o HDM.o HumanSchreck.o OVM.o VDIFF.o FPE.o VDT.o CDDA.o Kerner.o Gipps.o KernerCA.o PTmodel.o ASGM.o ADAS.o NH.o CACC.o PCF.o LCM.o BIDM.o RegionControl.o

LIBOBJECTS=general.o InOut.o RandomUtils.o

#####################################################################
# compiler specific settings
#####################################################################

CFLAGS= -Wall -O3

## gnu compiler: Folgende zwei Zeilen aktivieren
CC=g++
CLINK=-lm

## intel compiler nur auf vwitme010 installiert
##explizit statisch linken damit binary auch auf anderen maschinen ausfuehrbar.
## dafuer die naechsten zwei optionen aktivieren:
#CC=/opt/intel/cc/9.1.044/bin/icpc  
#CLINK=-lm -i-static -L/opt/intel/cc/9.1.044/lib

#####################################################################
# targets:
#####################################################################


mic : $(OBJECTS) $(LIBOBJECTS)
	${CC} -o ${BINDIR}/${OUTNAME} -O3 $(OBJECTS) $(LIBOBJECTS) ${CLINK}

all : mic

# suffix regel: mache aus *.cpp ein *.o, und zwar fuer eingabedatei $<
#.cpp.o:
#	${CC} $(CFLAGS) -I ${LIBDIR} -c $< -o $(OBJDIR)/$@
.cpp.o:
	${CC} $(CFLAGS) -I ${LIBDIR} -c $<


# lib files 

general.o: ${LIBDIR}/general.cpp
	${CC} $(CFLAGS) -c ${LIBDIR}/general.cpp -o ${OBJDIR}/general.o

InOut.o: ${LIBDIR}/InOut.cpp
	${CC}  $(CFLAGS) -c ${LIBDIR}/InOut.cpp -o $(OBJDIR)/InOut.o

RandomUtils.o: ${LIBDIR}/RandomUtils.cpp
	${CC} $(CFLAGS) -c ${LIBDIR}/RandomUtils.cpp -o $(OBJDIR)/RandomUtils.o


#
# Misc. targets
#
clean :
	rm -v $(OBJDIR)/*.o

help :
	@ echo "The following make targets are available"
	@ echo " help - print this message"
	@ echo " mic - make everything"
	@ echo " clean - remove object files"



#-I directory
#Mit der -I-Compileroption kann man beim Übersetzen eines Programmes die Liste der Verzeichnisse erweitern, in denen nach einer Datei gesucht wird.
# gcc -Iinc hello.c
#sucht nach stdio.h zuerst als inc/stdio.h, und erst dann als /usr/include/stdio.h.


#The -I option or the INCLUDE_PATH variable described below should always be used to set the include path for header files.
