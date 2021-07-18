# Backup Environment
This folder is intended to hold a backup virtual environment that can be used by any of the missions if they don't have a virtual environment of their own on the drone. This helps avoid circumstances where we go to fly the drones and have to abort because the virtual environment has not been set up.

## Usage
Create and set up virtual environment (only needs to be done once)
```bash
$ virtualenv env
$ ./env/bin/activate
$ make bootstrap
```

Activate virtual environment
```bash
$ ./env/bin/activate
```

The backup environment will also be activated if you use the run-mission-backup script
Run the following from the mission directory to activate the backup environment and run the mission.py script
```bash
$ ../bin/run-mission-backup.sh
```

## Maintinence
When a new mission has a new requirement that isn't required by any of the other missions, it will need to be added to the requirements.txt of this folder so that the backup-env will work with that mission.
After the requirement is added, `make bootstrap` must be run again in order to install the new dependencies.