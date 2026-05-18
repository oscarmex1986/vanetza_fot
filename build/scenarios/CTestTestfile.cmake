# CMake generated Testfile for 
# Source directory: /home/vagrant/artery/scenarios
# Build directory: /home/vagrant/artery/build/scenarios
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(mtits2017-test "/opt/omnetpp/bin/opp_run_dbg" "-n" "/home/vagrant/artery/src/artery:/home/vagrant/artery/src/traci:/home/vagrant/artery/extern/veins/examples/veins:/home/vagrant/artery/extern/veins/src/veins:/home/vagrant/artery/extern/inet/src:/home/vagrant/artery/extern/inet/examples:/home/vagrant/artery/extern/inet/tutorials:/home/vagrant/artery/extern/inet/showcases" "-l" "/home/vagrant/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/vagrant/artery/build/scenarios/highway-police/libartery_police.so" "-l" "/home/vagrant/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/vagrant/artery/build/src/artery/storyboard/libartery_storyboard.so" "-l" "/home/vagrant/artery/build/extern/libINET.so" "-l" "/home/vagrant/artery/build/extern/libveins.so" "-l" "/home/vagrant/artery/build/src/traci/libtraci.so" "-l" "/home/vagrant/artery/build/src/artery/libartery_core.so" "omnetpp.ini" "-uCmdenv" "--sim-time-limit=30s")
set_tests_properties(mtits2017-test PROPERTIES  WORKING_DIRECTORY "/home/vagrant/artery/scenarios/mt-its2017")
add_test(car2car-grid-cam_bsp "/opt/omnetpp/bin/opp_run_dbg" "-n" "/home/vagrant/artery/src/artery:/home/vagrant/artery/src/traci:/home/vagrant/artery/extern/veins/examples/veins:/home/vagrant/artery/extern/veins/src/veins:/home/vagrant/artery/extern/inet/src:/home/vagrant/artery/extern/inet/examples:/home/vagrant/artery/extern/inet/tutorials:/home/vagrant/artery/extern/inet/showcases" "-l" "/home/vagrant/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/vagrant/artery/build/scenarios/highway-police/libartery_police.so" "-l" "/home/vagrant/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/vagrant/artery/build/src/artery/storyboard/libartery_storyboard.so" "-l" "/home/vagrant/artery/build/extern/libINET.so" "-l" "/home/vagrant/artery/build/extern/libveins.so" "-l" "/home/vagrant/artery/build/src/traci/libtraci.so" "-l" "/home/vagrant/artery/build/src/artery/libartery_core.so" "omnetpp.ini" "-uCmdenv" "-ccam_bsp" "--sim-time-limit=30s")
set_tests_properties(car2car-grid-cam_bsp PROPERTIES  WORKING_DIRECTORY "/home/vagrant/artery/scenarios/car2car-grid")
subdirs("artery")
subdirs("gemv2")
subdirs("highway-police")
subdirs("rsu_grid")
subdirs("storyboard")
