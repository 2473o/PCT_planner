ROOT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd -P)


. $ROOT_DIR/.venv/bin/activate
source $ROOT_DIR/install/setup.bash
export PYTHONPATH=$PYTHONPATH:$ROOT_DIR/.venv/lib/python$(cat $ROOT_DIR/.python-version)/site-packages

ros2 launch pct_planner planner.launch.py
