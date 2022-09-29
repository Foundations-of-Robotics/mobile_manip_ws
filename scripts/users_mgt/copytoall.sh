#!/bin/bash
declare -i cnt=0
declare -i exist=0
declare -a us=""
declare -a jupus=""
#us=$1

#for d in /home/ens/mec745g*/ ; do
for d in /home/mecbotg*/ ; do
	echo "$d"
	# Get user name from path
	us="${d%/*}"
	us="${us##*/}"
	# Get username in lowercase (jupyter)
	jupus="${us,,}"
	echo "$us $jupus"


#	cp /home/admin_mec/mobile_manip_ws/scripts/start_remoteviz.sh $d/start_remoteviz.sh
#	chown $us:$us ${d}/start_remoteviz.sh

	rsync -av /home/admin_mec/mecbot-assignments/Project4/ /home/jupyter-$jupus/Project4/ --exclude Solutions
#	rm -R /home/jupyter-$jupus/.local
#	cp -r .local /home/jupyter-$jupus/

	chown -R jupyter-$jupus:jupyter-$jupus /home/jupyter-$jupus
	chmod -R g+rwx /home/jupyter-$jupus
	chmod -R o-rwx /home/jupyter-$jupus

done


#	git clone -b students https://git.initrobots.ca/dastonge/mecbot-assignments.git /home/jupyter-$us
