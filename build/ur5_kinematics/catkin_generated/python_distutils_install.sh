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

echo_and_run cd "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/jiaid/ur5_rehab_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/jiaid/ur5_rehab_ws/install/lib/python3/dist-packages:/home/jiaid/ur5_rehab_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/jiaid/ur5_rehab_ws/build" \
    "/usr/bin/python3" \
    "/home/jiaid/ur5_rehab_ws/src/ur5_kinematics/setup.py" \
     \
    build --build-base "/home/jiaid/ur5_rehab_ws/build/ur5_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/jiaid/ur5_rehab_ws/install" --install-scripts="/home/jiaid/ur5_rehab_ws/install/bin"
