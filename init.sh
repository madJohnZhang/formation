#!/bin/bash

gnome-terminal -x bash -c "expect commu.sh; exec bash"
gnome-terminal -x bash -c "expect track.sh; exec bash"
gnome-terminal -x bash -c "expect rostopic.sh; exec bash"