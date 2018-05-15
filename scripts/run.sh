#! /bin/bash
#
# Script for running opencog with HEAD.
#
#

set -e
_session_name="opencog"

# hrtool workspace
HR_WORKSPACE="$(hr env | grep HR_WORKSPACE | cut -d = -f 2)"

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
    guile -l load-opencog.scm;
    $SHELL"

  # Start ghost_bridge
  tmux new-window -t "$_session_name:" -n "ghost_bridge" \
    "roslaunch ghost_bridge run.launch;
    $SHELL"

  # Start a shell to cogserver
  tmux new-window -t "$_session_name:" -n "cogserver-shell" \
    "while ! nc -z localhost 17001; do sleep 0.1; done
    rlwrap telnet localhost 17001;
    $SHELL"

  echo "Finished starting opencog services in a new background tmux session"
}

# Start opencog tmux session
tmux has-session -t "$_session_name" || start_opencog_tmux_session
tmux a -t "$_session_name"