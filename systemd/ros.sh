#/bin/bash 

case "$1" in 
		start)
				echo "Iniciando ROS..."
				. /home/pi/ROS/gpar-robot-ros/devel/setup.sh
				roslaunch gpar_nanook nanook.launch &
				echo "Ros iniciado!"
				exit 0 #0 eh bom !
				;;

		stop)
				echo "Ros parando"
				pkill -SIGINT roscore
				pkill -SIGINT roslaunch
				;;
		*)
				echo "DEFAULT"
				exit 1
				;;
esac

exit 0
