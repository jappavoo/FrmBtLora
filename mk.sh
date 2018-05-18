#!/bin/bash

usage="USAGE: $0 <path to arduino-builder> [cmd]
   cmd: compile|dis  default is compile"

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
  if [[ -d build ]]; then
    rm -rf build
  fi
  mkdir build

#	    -prefs=compiler.cpp.extra_flags="-DJADEF" \
#	    -prefs=compiler.c.extra_flags="-DJADEF" \
    
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
	    -prefs=build.extra_flags="-fstack-usage -Xlinker -Map=$sketch.map" \
	    $src
fi

if [[ $cmd == upload ]]; then
    port=$3
    baud=115200
    if [[ -z $port ]]; then
	echo "upload requires that you also pass in the port you would have selected in the ide: eg. /dev/cu.usbmodem141411"
	exit -1
    fi
    $0 $1 compile
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
fi

if [[ $cmd == dis ]]; then
    if [[ ! -a build/$elf ]]; then
	echo "ERROR: can't find build/$elf.  Try compiling first"
	exit -1
    fi   
    $toolsToolChain/bin/avr-objdump -dS build/$elf > ${sketch}.dis
fi

if [[ $cmd == nm ]]; then
    if [[ ! -a build/$elf ]]; then
	echo "ERROR: can't find build/$elf.  Try compiling first"
	exit -1
    fi   
    $toolsToolChain/bin/avr-nm -C --print-size --size-sort --line-numbers build/$elf > ${sketch}.nm
fi

	  
