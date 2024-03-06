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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/alu/Escritorio/VAR/P1/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/alu/Escritorio/VAR/P1/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/alu/Escritorio/VAR/P1/install/lib/python3/dist-packages:/home/alu/Escritorio/VAR/P1/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/alu/Escritorio/VAR/P1/build" \
    "/usr/bin/python3" \
    "/home/alu/Escritorio/VAR/P1/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/alu/Escritorio/VAR/P1/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/alu/Escritorio/VAR/P1/install" --install-scripts="/home/alu/Escritorio/VAR/P1/install/bin"
