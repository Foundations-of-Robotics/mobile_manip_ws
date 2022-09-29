#!/bin/bash
declare -a us=""
# fetch automatically the station IP address from its network adapter wlp3s0
declare -a ip=`ifconfig wlp3s0 | awk '/inet / {print $2}'`

for d in /home/mecbotg*/ ; do
	echo "$d"
	us="${d%/*}"
	us="${us##*/}"

#	rm -R /home/jupyter-$us
	cp .bashrc ${d}.bashrc
	echo "ROS_IP="$ip >> ${d}.bashrc
done

rm groups_no.csv
touch groups_no.csv
