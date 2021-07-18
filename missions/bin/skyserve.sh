# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

cd ~/skyserve
. env/bin/activate
FC_ADDR=tcp:127.0.0.1:5760 make serve
