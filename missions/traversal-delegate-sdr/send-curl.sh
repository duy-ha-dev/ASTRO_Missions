# send-curl.sh for Delegate Traversal SDR mission

# Mission Parameters:
corners="stadium"
alt="[5, 10, 15, 20]"
airspeed=5

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
 	"corners": "$corners",
  	"alt": $alt,
        "airspeed": $airspeed
}
EOF
}

# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

curl $drone_ip:$mission_port/start-mission --data "$(generate_post_data)"
