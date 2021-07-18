# send-curl.sh for Formation Traversal mission

# Mission Parameters:
region_name="wiess-2"
min_alt=7
hover_duration=5
speed=2


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
 	"region_name": "$region_name", 
  	"min_alt": $min_alt, 
  	"hover_duration": $hover_duration,
  	"speed": $speed
}
EOF
}

# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

curl $drone_ip:$mission_port/start-mission --data "$(generate_post_data)"
