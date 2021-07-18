# send-curl.sh for Air Spectrum Cheater mission

# Mission Parameters:
region_id="stadium"
alt="[10, 13]"
airspeed=3
use_formation="true"
hover_time=30.0


# ===============================================
# Below should only change for different missions
mission_port=4000

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
    "region_id": "$region_id",
    "alt": $alt,
    "airspeed": $airspeed,
    "type": "$type",
    "hover_time": $hover_time,
    "use_formation":$use_formation
}
EOF
}

# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

curl $drone_ip:$mission_port/start-mission --data "$(generate_post_data)"
