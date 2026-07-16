#!/bin/bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"


if [ $# -eq 0 ]; then
  echo "Usage: $0 {s r}"
  echo "./run_mc.sh s          [this command will run sim]"
  echo "./run_mc.sh r          [this command will run robot]"
  exit 1
fi

case $1 in 
r)
    echo "run robot"
    export LD_LIBRARY_PATH=${DIR}/build/export/mc/bin
    export ROBOT_TYPE=XG
    cd ${DIR}/build/export/mc/bin/
    taskset -c 7 ./mc_ctrl r
    ;;
*)
    echo "invalid params: $1"
    echo "use params: $0 {r}"
    exit 1
    ;;
esac

export ROBOT_TYPE=XG
