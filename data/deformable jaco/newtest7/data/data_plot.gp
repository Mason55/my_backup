#set term png
#set output "plot.png"

#set terminal postscript eps color "Time-Roman" 30
#set output 'try_plot'
#set xrange [0:5]

set terminal x11 1
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angle [deg]" 
plot  'data.theta'	using 1:2 with lines title "Joint angle 1",\
        'data.rtheta'	using 1:2 with lines title "Reference angle 1"

set term x11 2
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angle [deg]" 
plot  'data.theta'	using 1:3 with lines title "Joint angle 2",\
        'data.rtheta'	using 1:3 with lines title "Reference angle 2"

set term x11 3
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angle [deg]" 
plot  'data.theta'	using 1:4 with lines title "Joint angle 3",\
        'data.rtheta'	using 1:4 with lines title "Reference angle 3"

set term x11 4
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angle [deg]" 
plot  'data.theta'	using 1:5 with lines title "Joint angle 4",\
        'data.rtheta'	using 1:5 with lines title "Reference angle 4"

set term x11 5
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angle [deg]" 
plot  'data.theta'	using 1:6 with lines title "Joint angle 5",\
        'data.rtheta'	using 1:6 with lines title "Reference angle 5"

set term x11 6
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angle [deg]" 
plot  'data.theta'	using 1:7 with lines title "Joint angle 6",\
        'data.rtheta'	using 1:7 with lines title "Reference angle 6"

set term x11 7
#set term postscript eps enhanced color "Time-Roman"
set xlabel "Time [sec]"
set ylabel "Angular velocity [deg]" 
plot  'data.dtheta'	using 1:2 with lines title "Joint 1 velocigy",\
         'data.dtheta'	using 1:3 with lines title "Joint 2 velocigy",\
         'data.dtheta'	using 1:4 with lines title "Joint 3 velocigy",\
         'data.dtheta'	using 1:5 with lines title "Joint 4 velocigy",\
         'data.dtheta'	using 1:6 with lines title "Joint 5 velocigy",\
         'data.dtheta'	using 1:7 with lines title "Joint 6 velocigy"


