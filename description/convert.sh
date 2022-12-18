#!/bin/bash
xacro ardak/ardak.urdf > model.urdf
gz sdf -p model.urdf > model.sdf