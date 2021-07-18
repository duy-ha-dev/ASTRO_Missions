# -e: Exit as soon as any command fails
# -x: Display commands as they are run
set -ex

. ../backup-env/env/bin/activate
python mission.py --fc-addr tcp:127.0.0.1:5760
