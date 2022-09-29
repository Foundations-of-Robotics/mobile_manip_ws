#!/bin/bash
declare -i cnt=0
declare -i exist=0
declare -a us=""
declare -a jupus=""
declare -a netdev=`ip route get 192.168.0.1 | sed -nr 's/.*dev ([^\ ]+).*/\1/p'`
declare -a ip=`ifconfig $netdev | awk '/inet / {print $2}'`

# get the new Id
while IFS=, read -r col1 col2
do
    cnt=$col1
#    echo "I got:$cnt|$col2"
done < groups_no.csv

for d in /home/mecbotg*/ ; do
	exist=0
	echo "$d"
	# Get user name from path
	us="${d%/*}"
	us="${us##*/}"
	# Get username in lowercase (jupyter)
	jupus="${us,,}"
	echo "$us $jupus"

	while IFS=, read -r col1 col2
	do
    		if [ $us == $col2 ]; then
			exist=1
		fi
	done < groups_no.csv

	if [ $exist -eq 1 ]; then
		echo "This user exist!"
	else
		echo "------> Using $cnt for new user $us"

	        # To clean jupyter folder
	       	rm -R /home/jupyter-$jupus
		mkdir /home/jupyter-$jupus
	       	rm -R /home/jupyter-$jupus/.local
	       	cp -r .local /home/jupyter-$jupus/
#$	       	cp -r /home/admin_mec/mecbot-assignments /home/jupyter-$jupus/
	       	rsync -av /home/admin_mec/mecbot-assignments/ /home/jupyter-$jupus/ --exclude Solutions

		# To set the user unique ports
	        cp .bashrc ${d}.bashrc
        	echo "ROS_IP="$ip >> ${d}.bashrc
		echo "export GAZEBO_MASTER_URI=http://localhost:1147"$cnt >> ${d}.bashrc
		echo "export ROS_MASTER_URI=http://localhost:1137"$cnt >> ${d}.bashrc
		echo "export GZWEB_PORT=808"$cnt >> ${d}.bashrc
		echo "export ROSBRIDGE_PORT=909"$cnt >> ${d}.bashrc

		# To get the gzweb launch script
		cp /home/admin_mec/mobile_manip_ws/scripts/start_gzweb.sh $d
		chown $us:$us ${d}/start_gzweb.sh

		# To launch with display (for cameras)
		cp /home/admin_mec/mobile_manip_ws/scripts/start_remoteviz.sh $d/start_remoteviz.sh
		chown $us:$us ${d}start_remoteviz.sh

		echo $cnt,$us >> groups_no.csv
		cnt=$cnt+1
	fi


	# To allow user and to access their jupyter folder
	usermod -a -G jupyter-$jupus $us
	usermod -a -G audio $us
	rm -R ${d}jupyter-notebooks
	ln -sf /home/jupyter-$jupus ${d}jupyter-notebooks
	chown -R $us:$us ${d}jupyter-notebooks
	chown -R jupyter-$jupus:jupyter-$jupus /home/jupyter-$jupus
	chmod -R g+rwx /home/jupyter-$jupus
	chmod -R o-rwx /home/jupyter-$jupus

done

tljh-config reload proxy
#	git clone -b students https://git.initrobots.ca/dastonge/mecbot-assignments.git /home/jupyter-$us
