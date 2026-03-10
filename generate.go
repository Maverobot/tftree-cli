// generate.go
package main

//go:generate go run github.com/PolibaX/rclgo/cmd/rclgo-gen generate -d msgs --include-package-deps tf2_msgs --message-module-prefix github.com/Maverobot/tftree-cli/msgs --ignore-ros-distro-mismatch --cgo-flags-path ""
