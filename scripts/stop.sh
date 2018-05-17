#! /bin/bash
#
# Script for killing the opencog session
#
#

set -e
_session_name="opencog"

tmux kill-session -t "$_session_name"