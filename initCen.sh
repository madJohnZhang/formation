#!/bin/bash

gnome-terminal -x bash -c "expect commuCen.sh; exec bash"
gnome-terminal -x bash -c "expect trackCen.sh; exec bash"
gnome-terminal -x bash -c "expect rostopicCen.sh; exec bash"