#!/bin/bash
declare -a us=""

for d in /home/mecbotg*/ ; do
	echo "$d"
	us="${d%/*}"
	us="${us##*/}"

	deluser --remove-home $us
	groupdel $us
	deluser --remove-home jupyter-$us
	groupdel jupyter-$us
	rm -R /home/jupyter-$us
#	tljh-config remove-item users.admin $us
	tljh-config remove-item users.allowed $us
done
tljh-config reload proxy
#rm groups_pwd.csv
rm groups_no.csv
