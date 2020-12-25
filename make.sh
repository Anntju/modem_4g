#! /bin/bash


	#echo $inputfile
	#echo $outputfile
	echo "complie $inputfile for board with cross complie tool"
	source /opt/poky/2.1.1/environment-setup-cortexa9hf-neon-poky-linux-gnueabi
	echo $CC
	#$CC -pthread -o ${outputfile} ${inputfile}
	echo "begin generate file !"
	make


