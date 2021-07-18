# send-curl.sh for Boomerang Mission
#For each waypoint, the drone will hover for $hover_duration. If you dont want that, just set to zero
#For each value of altitude (is a list!!), the drone will pass in all waypoints the number of loop times

# Mission Parameters:
waypoints='[{"lat": 29.716247, "lon": -95.409415}, {"lat": 29.716246, "lon": -95.409206}, {"lat": 29.716415, "lon":-95.409204}, {"lat": 29.716409, "lon": -95.409413}]'
hover_duration=10
air_speed=1.5
altitudes='[10]'
loops=2


# ===============================================
# Below should only change for different missions
mission_port=4002

if [ -z "$1" ]
  then
    echo "Error: name of drone not given"
    exit 1
fi

drone_name=$1
drone_ip=$(../bin/util/get-drone-ip.sh $drone_name)
echo "Will send curl to: $drone_name"

generate_post_data()
{
 	cat <<EOF
{
 	"waypoints": $waypoints,
  	"hover_duration": $hover_duration,
  	"air_speed": $air_speed,
  	"altitudes": $altitudes,
	"loops": $loops
}
EOF
}

# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

curl $drone_ip:$mission_port/start-mission --data "$(generate_post_data)"
