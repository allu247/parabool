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

echo_and_run cd "/home/test/Projects/parabol-controller/src/modbus/modbus"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/test/Projects/parabol-controller/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/test/Projects/parabol-controller/install/lib/python2.7/dist-packages:/home/test/Projects/parabol-controller/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/test/Projects/parabol-controller/build" \
    "/usr/bin/python2" \
    "/home/test/Projects/parabol-controller/src/modbus/modbus/setup.py" \
    build --build-base "/home/test/Projects/parabol-controller/build/modbus/modbus" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/test/Projects/parabol-controller/install" --install-scripts="/home/test/Projects/parabol-controller/install/bin"
