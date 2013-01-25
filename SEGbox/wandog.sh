#!/bin/sh

# SEGbox wifi manager

HOSTS="google.com"

for HOST in $HOSTS
do

    # 10 times, with a decent timeout. With output to suppressed

    ping -c 10 -w 2000 $HOST  > /dev/null

    if [ $? -ne 0 ]
      then
        echo -n "$HOST unreachable on " && date +%d/%m/%Y
        # Oh oh, no googles.  Lets re-jig the whole ntworks now.
        /etc/init.d/network restart
        sleep(10)
        wifi
    fi

done

exit 0
