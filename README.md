# tftree-cli

A lightweight, terminal-based ROS 2 TF tree viewer written in Go. Displays the transform frame hierarchy directly in your terminal using a folder-like tree structure with Unicode box-drawing characters.

Inspired by [tf_tree_terminal](https://github.com/Tanneguydv/tf_tree_terminal).

## Features

- **Terminal Based**: No GUI required — works over SSH, in containers, etc.
- **Tree Visualization**: Uses Unicode characters (`├──`, `└──`, `│`) to show parent-child relationships clearly.
- **Single Binary**: Distributed as a single static binary — no Python, no colcon build needed.
- **Stdin Mode**: Pipe data from `ros2 topic echo` for maximum flexibility.
- **File Export**: Optionally save the tree to a text file for documentation.
- **Cross-Platform**: Builds for Linux, macOS, and Windows (amd64 and arm64).

## Installation

### From Releases

Download the latest binary from the [Releases](https://github.com/Maverobot/tftree-cli/releases) page.

```bash
# Linux amd64
curl -L https://github.com/Maverobot/tftree-cli/releases/latest/download/tftree-cli_linux_amd64.tar.gz | tar xz
sudo mv tftree-cli /usr/local/bin/
```

### From Source

```bash
go install github.com/Maverobot/tftree-cli@latest
```

## Usage

### Live Mode (requires `ros2` CLI)

Subscribe to `/tf` and `/tf_static` topics for a duration (default 2 seconds) and display the tree:

```bash
tftree-cli
tftree-cli -duration 5s
```

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

In **live mode**, the tool runs `ros2 topic echo` on both `/tf` and `/tf_static` topics concurrently for the specified duration. It parses the streaming YAML output to extract `frame_id` → `child_frame_id` relationships.

In **stdin mode**, it reads YAML-formatted TF message data (as produced by `ros2 topic echo /tf`) from standard input.

In both modes, it builds a tree data structure from the parent-child relationships and renders it using a depth-first traversal with proper indentation and Unicode box-drawing characters.

## Distribution

Releases are automated via [GoReleaser](https://goreleaser.com/). To create a new release:

```bash
git tag v1.0.0
git push origin v1.0.0
```

This triggers the GitHub Actions release workflow, which builds binaries for all platforms and creates a GitHub release with downloadable archives.