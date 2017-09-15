# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.mpc.Debug:
/Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/Debug/mpc:
	/bin/rm -f /Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/Debug/mpc


PostBuild.mpc.Release:
/Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/Release/mpc:
	/bin/rm -f /Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/Release/mpc


PostBuild.mpc.MinSizeRel:
/Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/MinSizeRel/mpc:
	/bin/rm -f /Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/MinSizeRel/mpc


PostBuild.mpc.RelWithDebInfo:
/Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/RelWithDebInfo/mpc:
	/bin/rm -f /Users/Ollie/Documents/CarND/CarND-MPC-Project/IDE/xcode/RelWithDebInfo/mpc




# For each target create a dummy ruleso the target does not have to exist
