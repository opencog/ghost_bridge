#! /bin/bash
#
# Script for running opencog with HEAD.
#
#

set -e

_session_name="opencog"

# hrtool workspace
HR_WORKSPACE="$(hr env | grep HR_WORKSPACE | cut -d = -f 2)"

# Default launch file name
GB_LAUNCH_FILE='avatar.launch'

# Handle command-line arguments
for i in ${@}; do
  case $i in
    in-gdb)
      GDB="gdb -ex r --args"
      echo "Will run opencog in gdb" ;;
    enable-ecan)
      ECAN="-- enable-ecan"
      echo "Will start ecan" ;;
    robot)
      GB_LAUNCH_FILE='robot.launch';;
    *)
      echo "only 'in-gdb', 'enable-ecan' and 'robot' arguments are allowed"
      exit 1 ;;
  esac
done

# get robot name from rosparam
ROBOT_NAME="$(rosparam get /robot_name)"

# start opencog processes in tmux session.
start_opencog_tmux_session()
{
  echo "Start opencog services in a new background tmux session"

  # Start relex
  tmux new-session -d -s "$_session_name" -n "relex" \
    "cd $HR_WORKSPACE/OpenCog/relex &&
    bash opencog-server.sh;
    $SHELL"

  # Start the cogserver
  # TODO: catkin_find should be used to find the correct path as this will not work in non dev environments
  tmux new-window -t "$_session_name:" -n "cogserver" \
    "cd $HR_WORKSPACE/HEAD/src/ghost_bridge/scripts &&
    $GDB guile -l load-opencog.scm $ECAN;
    $SHELL"

  # Start ghost_bridge
  tmux new-window -t "$_session_name:" -n "ghost_bridge" \
    "roslaunch ghost_bridge $GB_LAUNCH_FILE robot_name:=$ROBOT_NAME;
    $SHELL"

  # Start a shell to cogserver
  tmux new-window -t "$_session_name:" -n "cogserver-shell" \
    "while ! nc -z localhost 17001; do sleep 0.1; done
    rlwrap telnet localhost 17001;
    $SHELL"

  # Start bash
  tmux new-window -t "$_session_name" -n "bash" \
    "bash;
    $SHELL"

  echo "Finished starting opencog services in a new background tmux session"
}

# Start opencog tmux session
tmux has-session -t "$_session_name" || start_opencog_tmux_session
tmux a -t "$_session_name"
