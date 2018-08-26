TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
LIBS +=-lpthread
SOURCES += \
    adrobot_control/joint.cpp \
    adrobot_control/servo.cpp \
    adrobot_etc/trajectory.cpp \
    adrobot_interface/dataExchange.cpp \
    adrobot_interface/dataSave.cpp \
    adrobot_interface/taskShell.cpp \
    adrobot_io/robot_state.cpp \
    adrobot_io/robot_state_RT.cpp \
    adrobot_io/ur_communication.cpp \
    adrobot_io/ur_driver.cpp \
    adrobot_io/ur_realtime_communication.cpp \
    adrobot_kinematics/ur_kine.cpp \
    adrobot_system/time.cpp \
    matrix/func.cpp \
    matrix/sample.cpp

HEADERS += \
    adrobot_control/adrobot_control.h \
    adrobot_etc/adrobot_etc.h \
    adrobot_interface/adrobot_interface.h \
    adrobot_io/do_output.h \
    adrobot_io/robot_state.h \
    adrobot_io/robot_state_RT.h \
    adrobot_io/ur_communication.h \
    adrobot_io/ur_driver.h \
    adrobot_io/ur_realtime_communication.h \
    adrobot_kinematics/adrobot_kinematics.h \
    adrobot_system/adrobot_system.h \
    matrix/matrix.h \
    common.h
