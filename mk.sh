#!/bin/bash
#  Jonathan Apppavoo
#  To use this script you must install the Arduino IDE and find
#  the path to the arduino-builder java program
#  To use the upload command you will need to also know what port you specify in the
#  IDE to upload to your arduino
#
#  convience script to build sketches in a controlled manner
#  that is still compatible with the Arduino IDE but
#  customizes compile flags and uses tool chain to provide
#  more control and details of the finaly binary
#  useful for debugging size issues
#  note to use the stack-usage info you will need to
#  compile individual files turing off flto

#  Example:
#   In the IDE create a project and put this script in the created project directory
#   open a terminal and run this script in that directory and run the script 
#   In te following lets assume you have created a MyProject.ino which the IDE places
#   into its own directory called MyProject. Which in my case ends up
#   in Documents/Arduino/MyProject.  Further I have installed the IDE into an Applications
#   directory in my home directory.  
#   eg.  $ cd ~/Documents/Arduino/MyProject
#        $ cp ~/mk.sh .
#        $ ./mk.sh ${HOME}Applications/Arduino-latest.app/Contents/Java/arduino-builder upload /dev/cu.usbmodem141411


usage="
USAGE: $0 <path to arduino-builder> [compile|upload|info|clean]
       default is compile
         compile
         upload  <serial port of arduino> [4 character id]"

sketch=$PWD
sketch=${sketch##*/}.ino
elf=${sketch}.elf
hex=${sketch}.hex
src=${PWD}/${sketch}

abPath=$1

cmd=${2:-compile}

echo "$0: Executing: $cmd"

javadir=${abPath%/*}

hardware=${javadir}/hardware
toolsBuilder=${javadir}/tools-builder
toolsToolChain=${hardware}/tools/avr
librariesBuiltIn=${javadir}/libraries
libraries=${PWD}/../libraries
builddir=${PWD}/build


if [[ -z $sketch || ! -a $sketch ]]; then
    echo "ERROR: failed to find $src: Please run script in the ardunio project directory containing the sketch"
    echo $usage
    exit -1
fi

if [[ ! -d $libraries ]]; then
    echo "ERROR:  can't locate your libraries directory $libraries.  Be sure to check out this source in your Arduino sketch folder"
    echo $usage
    exit -1
fi

if [[ -z $abPath ]]; then
    echo $usage
    exit -1
fi


if [[ $cmd == compile ]]; then
    $0 $1 clean
    mkdir build

    id=$3
    
    if [[ ! -z $id ]]; then
	if (( ${#id} != 4 )); then
	    echo "ERROR:  id must be a 4 characters long"
	    echo $usage
	    exit -1
	fi
	echo "Image will be compiled to set box ID to $id"
	MID="\"-DFARM_BEATS_ID={'${id:3:1}','${id:2:1}','${id:1:1}','${id:0:1}'}\""
    fi
#	    -prefs=compiler.cpp.extra_flags="-DJADEF" \
#	    -prefs=compiler.c.extra_flags="-DJADEF" \

    ${abPath} -dump-prefs \
	    -hardware ${hardware} \
	    -tools ${toolsBuilder} \
	    -tools ${toolsToolChain} \
	    -built-in-libraries ${librariesBuiltIn} \
	    -libraries ${libraries} \
	    -fqbn=arduino:avr:uno \
	    -vid-pid=0X2341_0X0042 \
	    -build-path ${builddir} \
	    -warnings=all \
	    -prefs=build.warn_data_percentage=75 \
	    -prefs=runtime.tools.arduinoOTA.path=${toolsToolChain} \
	    -prefs=runtime.tools.avr-gcc.path=${toolsToolChain} \
	    -prefs=runtime.tools.avrdude.path=${toolsToolChain} \
	    -verbose \
	    -prefs=build.extra_flags="-fstack-usage $MID" \
	    -prefs=compiler.c.elf.extra_flags="-Xlinker -Map=$sketch.map" \
	    $src > ${sketch}.prefs

  ${abPath} -compile \
	    -hardware ${hardware} \
	    -tools ${toolsBuilder} \
	    -tools ${toolsToolChain} \
	    -built-in-libraries ${librariesBuiltIn} \
	    -libraries ${libraries} \
	    -fqbn=arduino:avr:uno \
	    -vid-pid=0X2341_0X0042 \
	    -build-path ${builddir} \
	    -warnings=all \
	    -prefs=build.warn_data_percentage=75 \
	    -prefs=runtime.tools.arduinoOTA.path=${toolsToolChain} \
	    -prefs=runtime.tools.avr-gcc.path=${toolsToolChain} \
	    -prefs=runtime.tools.avrdude.path=${toolsToolChain} \
	    -verbose \
	    -prefs=build.extra_flags="-fstack-usage $MID" \
	    -prefs=compiler.c.elf.extra_flags="-Xlinker -Map=$sketch.map -Xlinker --cref" \
	    $src
  $0 $1 info
fi

if [[ $cmd == upload ]]; then
    port=$3
    id=$4
    baud=115200
    if [[ -z $port ]]; then
	echo "upload requires that you also pass in the port you would have selected in the ide: eg. /dev/cu.usbmodem141411"
	exit -1
    fi
    if [[ ! -z $id ]]; then
	echo "compiling uploading with hardcode id=$id to write eeprom"
    fi

    $0 $1 compile $id
    if [[ ! -a build/$hex ]]; then
	echo "ERROR: can't find build/$hex.  Try first getting a clean compile ;-)"
	exit -1
    fi
    $toolsToolChain/bin/avrdude \
	 -C${toolsToolChain}/etc/avrdude.conf \
	 -v \
	 -patmega328p \
	 -carduino \
	 -P${port} \
	 -b${baud} \
	 -D \
	 -Uflash:w:build/$hex:i
    if [[ ! -z $id ]]; then
       echo "sleep 5s for id to be written to eeprom"
       sleep 5
       echo "Recompiling and uploading final binary WITHOUT hardcoded ID"
       $0 $1 upload $port
    fi
fi

if [[ $cmd == info ]]; then
    if [[ ! -a build/$elf ]]; then
	echo "ERROR: can't find build/$elf.  Try compiling first"
	exit -1
    fi
    echo "dumping sizes"
    $toolsToolChain/bin/avr-size build/$elf > ${sketch}.sizes
    echo "Disassmbling"
    $toolsToolChain/bin/avr-objdump -dS build/$elf > ${sketch}.dis
    echo "Extracting symbol table"
    $toolsToolChain/bin/avr-readelf -s build/$elf > ${sketch}.sym
    echo "Running nm"
    $toolsToolChain/bin/avr-nm -C --print-size --size-sort --line-numbers build/$elf > ${sketch}.nm
    echo "Running nm for data only"
    $toolsToolChain/bin/avr-nm -Crtd --size-sort build/$elf  | grep -i ' [dbv] ' > ${sketch}.nmdata
    echo "Extracting dwarf info"
    $toolsToolChain/bin/avr-objdump --dwarf=info build/$elf > ${sketch}.dwarf
fi

	  
if [[ $cmd == clean ]]; then
    [[ -a build ]] && rm -rf build
    rm *.dis *.nm *.sym *.map *.dwarf *.prefs *.su *.sizes *.nmdata
fi
