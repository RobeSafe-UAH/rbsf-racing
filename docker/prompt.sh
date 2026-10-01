# Shell prompt for the RBSF Racing container. Sourced at the end of ~/.bashrc so
# it is not overridden by Ubuntu's default prompt.
# [jazzy·dev·D=0] roboracer@rbsf-racing:/ros2_ws ❯   (the domain is re-read on every prompt)
PS1='\[\e[2;36m\][${ROS_DISTRO}·${RBSF_FLAVOR}·D=${ROS_DOMAIN_ID:-0}]\[\e[0m\] \[\e[1;32m\]\u\[\e[0;37m\]@\[\e[1;33m\]rbsf-racing\[\e[0;37m\]:\[\e[1;34m\]\w\[\e[0m\] ❯ '
