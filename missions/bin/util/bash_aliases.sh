# The following should be in the ~/.bash_aliases file of the GCS
# sshpass must be installed on the GCS in order for the commands to run
# install it with 'sudo apt get install sshpass'

# SSH aliases for drones
# To use, simply type the name of the drone in the terminal
# Then the GCS will set the time on the target drone and open an ssh connection
alias hapi='sshpass -p ciaoastro ssh pi@172.27.0.130 "sudo date --set \"$(date)\"" && sshpass -p ciaoastro ssh pi@172.27.0.130'
alias terminator='sshpass -p ciaoastro ssh pi@172.27.0.160 "sudo date --set \"$(date)\"" && sshpass -p ciaoastro ssh pi@172.27.0.160'
alias conan='sshpass -p ciaoastro ssh pi@172.27.0.99 "sudo date --set \"$(date)\"" && sshpass -p ciaoastro ssh pi@172.27.0.99'
alias ivan='sshpass -p ciaoastro ssh pi@172.27.0.88 "sudo date --set \"$(date)\"" && sshpass -p ciaoastro ssh pi@172.27.0.88'
alias bobo='sshpass -p ciaoastro ssh pi@172.27.0.32 "sudo date --set \"$(date)\"" && sshpass -p ciaoastro ssh pi@172.27.0.32'
