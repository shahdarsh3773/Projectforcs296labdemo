#!/bin/bash
user=`ps axu | awk '{ if($11 == "./cs296_16_exe") print $1 }'`
id=`ps axu | awk '{ if($11 == "./cs296_16_exe") print $2}'`
currentID=`whoami`
if [[ $user == "" ]]
then
echo "Process is not running"
elif [[ $user == $currentID ]]
then
while true; do
    read -p "Do you wish to kill the base code process? " yn
    case $yn in
        [Yy]* ) kill $id; break;;
        [Nn]* ) exit;;
        * ) exit;;
    esac
done
else
echo "Current user does not have permission to terminate the process"
fi 

