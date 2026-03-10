# tftree-cli

A lightweight, terminal-based ROS 2 TF tree viewer written in Go. Displays the transform frame hierarchy directly in your terminal using a folder-like tree structure with Unicode box-drawing characters.

Inspired by [tf_tree_terminal](https://github.com/Tanneguydv/tf_tree_terminal).

## Features

- **Native ROS 2 Subscription**: Connects directly to `/tf` and `/tf_static` topics using rclgo — no `ros2` CLI required at runtime.
- **Live Updating Display**: Continuously refreshes the tree in your terminal as new transforms arrive, with debounced rendering.
- **Terminal Based**: No GUI required — works over SSH, in containers, etc.
- **Tree Visualization**: Uses Unicode characters (`├──`, `└──`, `│`) to show parent-child relationships clearly.
- **AppImage Distribution**: Distributed as a self-contained AppImage with bundled ROS 2 libraries — no ROS 2 installation needed.
- **Stdin Mode**: Pipe data from `ros2 topic echo` for maximum flexibility.
- **File Export**: Optionally save the tree to a text file for documentation.

## Installation

### AppImage (Recommended)

Download the latest AppImage from the [Releases](https://github.com/Maverobot/tftree-cli/releases) page. No ROS 2 installation required.

```bash
curl -L https://github.com/Maverobot/tftree-cli/releases/latest/download/tftree-cli-x86_64.AppImage -o tftree-cli
chmod +x tftree-cli
sudo mv tftree-cli /usr/local/bin/
```

### From Source

Requires ROS 2 Jazzy and Go 1.24+ installed on your system.

```bash
git clone https://github.com/Maverobot/tftree-cli.git
cd tftree-cli
source /opt/ros/jazzy/setup.bash
go generate ./...
source cgo-flags.env
CGO_ENABLED=1 go build -v -o tftree-cli .
sudo mv tftree-cli /usr/local/bin/
```

## Usage

### Live Mode (default)

Natively subscribes to `/tf` and `/tf_static` topics and continuously updates the tree display:

```bash
tftree-cli
tftree-cli --refresh 1s
tftree-cli --node-name my_tftree
```

| Flag | Default | Description |
|------|---------|-------------|
| `--refresh` | `500ms` | Minimum interval between display refreshes |
| `--node-name` | `tftree_cli` | ROS 2 node name used for subscriptions |

### Stdin Mode

Pipe transform data from `ros2 topic echo`:

```bash
ros2 topic echo /tf | tftree-cli --stdin
```

### Save to File

```bash
tftree-cli tree.txt
tftree-cli --stdin tree.txt < tf_data.yaml
```

### Example Output

```
--- ROS 2 TF Tree Snapshot ---
map
└── odom
    └── base_footprint
        └── base_link
            ├── camera_link
            ├── chassis_link
            │   ├── front_axle
            │   └── rear_axle
            └── velodyne_link
------------------------------
```

## How It Works

In **live mode**, the tool creates a ROS 2 node and natively subscribes to both `/tf` and `/tf_static` topics using [rclgo](https://github.com/tiiuae/rclgo). Incoming transforms are collected and the tree display is refreshed with debounced rendering at the configured `--refresh` interval. On TTY terminals, ANSI escape codes are used to clear and redraw the display in place.

In **stdin mode**, it reads YAML-formatted TF message data (as produced by `ros2 topic echo /tf`) from standard input.

In both modes, it builds a tree data structure from the parent-child relationships and renders it using a depth-first traversal with proper indentation and Unicode box-drawing characters.

## Distribution

Releases are automated via [GoReleaser](https://goreleaser.com/). To create a new release:

```bash
git tag v1.0.0
git push origin v1.0.0
```

This triggers the GitHub Actions release workflow, which builds the Linux amd64 binary with CGO and creates a GitHub release with downloadable archives and an AppImage.