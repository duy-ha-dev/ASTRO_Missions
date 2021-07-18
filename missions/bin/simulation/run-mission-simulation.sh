# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

. env/bin/activate
python mission.py --fc-addr udp:localhost:6003