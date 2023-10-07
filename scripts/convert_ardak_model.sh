#!/bin/sh
if [ -z "$MDIR" ]; then
    MDIR=../description/ardak
fi


xacro $MDIR/ardak.urdf use_simulation:=true > $MDIR/model.urdf
ign sdf -p $MDIR/model.urdf > $MDIR/model.sdf

sed -i -e "/<model name='ardak'>/r $MDIR/depth_camera_sensor.sdf" $MDIR/model.sdf
#sed -i -e "/<model name='ardak'>/r $MDIR/gps.sdf" $MDIR/model.sdf
#sed -i -e "/<model name='ardak'>/r $MDIR/imu.sdf" $MDIR/model.sdf
