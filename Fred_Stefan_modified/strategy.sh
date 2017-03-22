#!/bin/sh
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIR="$DIR/libs/"
export LD_LIBRARY_PATH=$DIR
java -cp out/production/Fred_Stefan_modified/:libs/jssc.jar:libs/v4l4j.jar  strategy.Strategy
