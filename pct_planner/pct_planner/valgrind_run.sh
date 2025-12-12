ROOT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd)

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${ROOT_DIR}/../lib/build/src/common/smoothing
export PYTHONPATH=$PYTHONPATH:${ROOT_DIR}/../lib

valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes \
 python3 plan.py \
 --scene=Building > valgrind_report.txt 2>&1