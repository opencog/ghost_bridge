;
; ROS robot movement module.
;
; Provides interfaces for making physical movements, and
; for saying things.

(define-module (opencog movement))

(use-modules (opencog) (opencog atom-types) (opencog python))

(load "actions.scm")

; Try loading the python code from this directory; else got for the
; install directory.  This assumes that the the current directory
; is in the python sys.path.
;
; If roscore is not running, then the load will hang. Thus, to avoid the
; hang, we test to see if we can talk to roscore. If we cannot, then load
; only the debug interfaces.
;
;
(define-public (start-ros-movement-node)
 (python-eval "


try:
    import rosgraph

    # Throw an exception if roscore is not running.
    rosgraph.Master('/rostopic').getPid()

    # Exec the action_node
    exec (open('action_node.py').read())
    print('Loaded the OpenCog ROS Action API')

except Exception as e:
    print('Error loading the OpenCog Action API', e)
"))
