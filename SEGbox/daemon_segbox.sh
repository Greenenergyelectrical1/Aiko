#!/bin/sh

# SEGbox restarter, for running on the crontabs config following
# from here>> */2 * * * * /root/daemon_segbox.sh >/dev/null 2>&1

ps -ef | grep -v grep | grep segbox.lua
# if not found - equals to 1, start it

if [ $? -eq 1 ]
then
    /etc/init.d/segbox start
else
    echo "SEGbox is running!"
fi