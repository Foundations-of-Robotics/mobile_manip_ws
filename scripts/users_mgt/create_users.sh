#!/bin/bash
# Script to generate new user adn/or reset passwords
declare -a PASS=""
i="0"

while [ $i -lt 10 ] ; do
# get the new Id
	while IFS=, read -r col1 col2 ; do
		if [ $col1 = mecbotg$i ] ; then
			PASS=$col2
			echo "Password exists for mecbotg$i: $PASS"
		fi
	done < groups_pwd.csv
	if [ "$PASS" = "" ] ; then
		echo "Storing mecbotg$i with $PASS"
		PASS=$(cat /dev/urandom | tr -dc 'a-zA-Z0-9' | fold -w 8 | head -n 1)
		echo  "mecbotg$i,$PASS" >> groups_pwd.csv
	fi
	egrep "^mecbotg$i" /etc/passwd >/dev/null
	if [ $? -eq 0 ]; then
		echo "mecbotg$i exists!"
		echo -e "$PASS\n$PASS" | passwd mecbotg$i
	else
		echo "Creating mecbotg$i with $PASS"
		useradd -m -p $(openssl passwd -crypt $PASS) mecbotg$i --shell /bin/bash
		sudo tljh-config add-item users.allowed mecbotg$i
#		echo "  - mecbotg$i" >> /opt/tljh/config/config.yaml
		# launch jupyter?
	fi
	PASS=""
	i=$[$i+1]
done

tljh-config reload proxy
