#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/arkadiros/ROS/tello_catkin_ws/src/rpg_svo/rqt_svo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/arkadiros/ROS/tello_catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/arkadiros/ROS/tello_catkin_ws/install/lib/python2.7/dist-packages:/home/arkadiros/ROS/tello_catkin_ws/build/rqt_svo/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/arkadiros/ROS/tello_catkin_ws/build/rqt_svo" \
    "/usr/bin/python" \
    "/home/arkadiros/ROS/tello_catkin_ws/src/rpg_svo/rqt_svo/setup.py" \
    build --build-base "/home/arkadiros/ROS/tello_catkin_ws/build/rqt_svo" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/arkadiros/ROS/tello_catkin_ws/install" --install-scripts="/home/arkadiros/ROS/tello_catkin_ws/install/bin"
