drone_name=$1
drone_ip=""

if [ "$drone_name" == "docker" ]
then
	drone_ip="172.17.0.2"
elif [ "$drone_name" == "hapi" ]
then
	drone_ip="172.27.0.130"
elif [ "$drone_name" == "terminator" ]
then
	drone_ip="172.27.0.160"
elif [ "$drone_name" == "conan" ]
then
	drone_ip="172.27.0.99"
elif [ "$drone_name" == "ivan" ]
then
	drone_ip="172.27.0.88"
elif [ "$drone_name" == "bobo" ]
then
	drone_ip="172.27.0.32"
elif [ "$drone_name" == "fenomeno" ]
then
	drone_ip="172.27.0.9"
else
	echo "Drone name not recognized: " + $drone_name
fi

echo $drone_ip