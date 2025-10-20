sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1 2>/dev/null  true
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1 2>/dev/null  true
systemctl --user restart transitive-robot.service
systemctl --user status transitive-robot.service
