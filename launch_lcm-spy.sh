#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/lcm/imu
export CLASSPATH=/usr/share/java/lcm.jar:${DIR}/lcm/lcmtypes.jar:$CLASSPATH
pwd
lcm-spy