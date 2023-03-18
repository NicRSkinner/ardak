#!/bin/sh
if [ -z "$MDIR" ]; then
    MDIR=../description/ardak
fi


xacro $MDIR/ardak.urdf > $MDIR/model.urdf
gz sdf -p $MDIR/model.urdf > $MDIR/model.sdf

sed -i -e "/<model name='ardak'>/r $MDIR/realsense_camera.sdf" $MDIR/model.sdf
#sed -i -e "/<model name='ardak'>/r $MDIR/gps.sdf" $MDIR/model.sdf
#sed -i -e "/<model name='ardak'>/r $MDIR/imu.sdf" $MDIR/model.sdf
