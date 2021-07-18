# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

cd ~/skyserve
. env/bin/activate
FC_ADDR=udp:localhost:6002 make serve