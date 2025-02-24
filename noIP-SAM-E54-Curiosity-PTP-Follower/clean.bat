@echo off
rmdir firmware\demo_LAN865x_PTP_Follower.X\.generated_files /S/Q
rmdir firmware\demo_LAN865x_PTP_Follower.X\debug /S/Q
rmdir firmware\demo_LAN865x_PTP_Follower.X\build /S/Q
rmdir firmware\demo_LAN865x_PTP_Follower.X\report /S/Q
rmdir firmware\demo_LAN865x_PTP_Follower.X\dist\default /S/Q
rmdir firmware\demo_LAN865x_PTP_Follower.X\dist\release\debug /S/Q
rmdir firmware\demo_LAN865x_PTP_Follower.X\nbproject\private /S/Q
del   firmware\demo_LAN865x_PTP_Follower.X\nbproject\Makefile* /F/Q
del   firmware\demo_LAN865x_PTP_Follower.X\nbproject\Package* /F/Q

rmdir libtc6.X\.generated_files /S/Q
rmdir libtc6.X\debug /S/Q
rmdir libtc6.X\report /S/Q
rmdir libtc6.X\build /S/Q
rmdir libtc6.X\dist\default /S/Q
rmdir libtc6.X\dist\release\debug /S/Q
rmdir libtc6.X\nbproject\private /S/Q
del   libtc6.X\nbproject\Makefile* /F/Q
del   libtc6.X\nbproject\Package* /F/Q

pause