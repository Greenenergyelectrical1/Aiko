#!/bin/sh

echo Restarts all the networks for the SEGbox, bring wifi down...
wifi down
sleep 3
echo restarting the networks...
/etc/init.d/network restart
sleep 3
echo wifi coming up
wifi up
echo all done!



