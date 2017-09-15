#!/bin/sh

#  build.sh
#  MPC
#
#  Created by Ollie Steiner on 15/09/17.
#
mkdir build
cd build
cmake .. & make
open ../../term2_sim_mac/term2_sim.app
./mpc
