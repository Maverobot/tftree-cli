# Native TF Subscription & AppImage Release Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the subprocess-based TF collection with native rclgo ROS 2 subscriptions, add continuous live display, and create a GitHub Actions pipeline for AppImage distribution.

**Architecture:** Use PolibaX/rclgo (Jazzy-compatible Go bindings) to create a ROS 2 node that subscribes to `/tf` and `/tf_static`. The subscriber owns a thread-safe `parentOf` map and invokes an `onChange` callback with snapshot copies. `main.go` drives debounced terminal rendering using ANSI clear codes (with TTY detection). The existing `--stdin` mode is preserved. The AppImage workflow builds on Ubuntu 24.04 with ROS 2 Jazzy, bundles shared libraries, and uploads the artifact.

**Tech Stack:** Go 1.24, PolibaX/rclgo, rclgo-gen (message generation), tf2_msgs, GitHub Actions, linuxdeploy/appimagetool

**Spec:** `docs/superpowers/specs/2026-03-10-native-tf-subscription-appimage-design.md`

---

## Chunk 1: Project Setup & Message Generation

### Task 1: Add rclgo dependency and tools scaffolding

**Files:**
- Modify: `go.mod`
- Create: `tools.go`
- Create: `generate.go`

- [ ] **Step 1: Create tools.go for rclgo-gen dependency**

```go
// tools.go
//go:build tools

package main

import _ "github.com/PolibaX/rclgo/cmd/rclgo-gen"
```

- [ ] **Step 2: Create generate.go with go:generate directive**

```go
// generate.go
package main

//go:generate go run github.com/PolibaX/rclgo/cmd/rclgo-gen generate -d msgs --include-go-package-deps ./...
```

- [ ] **Step 3: Add rclgo dependency to go.mod**

Run:
```bash
source /opt/ros/jazzy/setup.bash
go get github.com/PolibaX/rclgo@latest
go mod tidy
```

Expected: `go.mod` updated with rclgo dependency. `go.sum` updated.

- [ ] **Step 4: Generate ROS 2 message bindings**

Run:
```bash
source /opt/ros/jazzy/setup.bash
go generate ./...
```

Expected: `msgs/` directory created with generated Go types for `tf2_msgs`, `geometry_msgs`, `std_msgs`, `builtin_interfaces`, and their dependencies.

- [ ] **Step 5: Create cgo-flags.env**

Create `cgo-flags.env` with the CGO flags needed for Jazzy:
```bash
export CGO_CFLAGS='-I/opt/ros/jazzy/include/action_msgs -I/opt/ros/jazzy/include/builtin_interfaces -I/opt/ros/jazzy/include/rcl -I/opt/ros/jazzy/include/rcl_action -I/opt/ros/jazzy/include/rcl_yaml_param_parser -I/opt/ros/jazzy/include/rcutils -I/opt/ros/jazzy/include/rmw -I/opt/ros/jazzy/include/rosidl_dynamic_typesupport -I/opt/ros/jazzy/include/rosidl_runtime_c -I/opt/ros/jazzy/include/rosidl_typesupport_interface -I/opt/ros/jazzy/include/service_msgs -I/opt/ros/jazzy/include/std_msgs -I/opt/ros/jazzy/include/type_description_interfaces -I/opt/ros/jazzy/include/unique_identifier_msgs -I/opt/ros/jazzy/include/geometry_msgs -I/opt/ros/jazzy/include/tf2_msgs'
export CGO_LDFLAGS='-L/opt/ros/jazzy/lib -Wl,-rpath=/opt/ros/jazzy/lib'
```

- [ ] **Step 6: Verify generated message bindings compile**

Run:
```bash
source /opt/ros/jazzy/setup.bash
source cgo-flags.env
go build ./msgs/...
```

Expected: No compilation errors.

- [ ] **Step 7: Commit**

```bash
git add tools.go generate.go go.mod go.sum cgo-flags.env msgs/
git commit -m "feat: add rclgo dependency and generate tf2_msgs bindings

Add PolibaX/rclgo as dependency for native ROS 2 subscriptions.
Generate Go message bindings for tf2_msgs and dependencies using
rclgo-gen.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Chunk 2: TF Subscriber Implementation

### Task 2: Write the TFSubscriber with tests

**Files:**
- Create: `internal/tftree/subscriber.go`
- Create: `internal/tftree/subscriber_test.go`
- Modify: `internal/tftree/collector.go` (remove `CollectFromROS2` and `echoTopic`)

- [ ] **Step 1: Write the failing test for ProcessTFMessage**

The subscriber logic has two testable parts: (a) extracting frame pairs from a tf2_msgs/TFMessage, and (b) managing the parentOf map with thread safety. We test the extraction logic without rclgo by factoring it into a `ProcessTFMessage` function.

Create `internal/tftree/subscriber_test.go`:
```go
package tftree

import (
	"sync"
	"testing"
)

func TestProcessTFMessage(t *testing.T) {
	parentOf := make(map[string]string)
	var mu sync.Mutex

	transforms := []FrameRelation{
		{ParentFrameID: "map", ChildFrameID: "odom"},
		{ParentFrameID: "odom", ChildFrameID: "base_link"},
		{ParentFrameID: "base_link", ChildFrameID: "camera_link"},
	}

	var called int
	var lastSnapshot map[string]string
	onChange := func(snapshot map[string]string) {
		called++
		lastSnapshot = snapshot
	}

	ProcessTFMessage(transforms, parentOf, &mu, onChange)

	if called != 1 {
		t.Fatalf("expected onChange called 1 time, got %d", called)
	}
	if len(lastSnapshot) != 3 {
		t.Fatalf("expected 3 relations, got %d: %v", len(lastSnapshot), lastSnapshot)
	}
	expected := map[string]string{
		"odom":        "map",
		"base_link":   "odom",
		"camera_link": "base_link",
	}
	for child, parent := range expected {
		if lastSnapshot[child] != parent {
			t.Errorf("expected parent of %q = %q, got %q", child, parent, lastSnapshot[child])
		}
	}
}

func TestProcessTFMessageMerges(t *testing.T) {
	parentOf := make(map[string]string)
	var mu sync.Mutex

	var lastSnapshot map[string]string
	onChange := func(snapshot map[string]string) {
		lastSnapshot = snapshot
	}

	// First message
	ProcessTFMessage([]FrameRelation{
		{ParentFrameID: "map", ChildFrameID: "odom"},
	}, parentOf, &mu, onChange)

	// Second message adds more frames
	ProcessTFMessage([]FrameRelation{
		{ParentFrameID: "odom", ChildFrameID: "base_link"},
	}, parentOf, &mu, onChange)

	if len(lastSnapshot) != 2 {
		t.Fatalf("expected 2 relations after merge, got %d: %v", len(lastSnapshot), lastSnapshot)
	}
	if lastSnapshot["odom"] != "map" || lastSnapshot["base_link"] != "odom" {
		t.Errorf("unexpected snapshot: %v", lastSnapshot)
	}
}

func TestProcessTFMessageSkipsEmpty(t *testing.T) {
	parentOf := make(map[string]string)
	var mu sync.Mutex

	// Transforms with empty frame IDs should be skipped
	transforms := []FrameRelation{
		{ParentFrameID: "", ChildFrameID: "odom"},
		{ParentFrameID: "map", ChildFrameID: ""},
	}

	var called int
	onChange := func(snapshot map[string]string) {
		called++
	}

	ProcessTFMessage(transforms, parentOf, &mu, onChange)

	if called != 1 {
		t.Fatalf("expected onChange called once, got %d", called)
	}
	if len(parentOf) != 0 {
		t.Errorf("expected no relations for empty frame IDs, got %v", parentOf)
	}
}

func TestProcessTFMessageConcurrency(t *testing.T) {
	parentOf := make(map[string]string)
	var mu sync.Mutex
	onChange := func(snapshot map[string]string) {}

	var wg sync.WaitGroup
	for i := 0; i < 100; i++ {
		wg.Add(1)
		go func(i int) {
			defer wg.Done()
			transforms := []FrameRelation{
				{ParentFrameID: "map", ChildFrameID: "odom"},
			}
			ProcessTFMessage(transforms, parentOf, &mu, onChange)
		}(i)
	}
	wg.Wait()

	mu.Lock()
	defer mu.Unlock()
	if parentOf["odom"] != "map" {
		t.Errorf("expected parent of odom = map, got %q", parentOf["odom"])
	}
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source cgo-flags.env && go test -v -run 'TestProcessTFMessage' ./internal/tftree/
```

Expected: FAIL — `ProcessTFMessage` and `FrameRelation` are undefined.

- [ ] **Step 3: Write the subscriber implementation**

Create `internal/tftree/subscriber.go`:
```go
package tftree

import (
	"context"
	"fmt"
	"sync"

	tf2_msgs_msg "github.com/Maverobot/tftree-cli/msgs/tf2_msgs/msg"
	"github.com/PolibaX/rclgo/pkg/rclgo"
)

// FrameRelation represents a parent→child TF frame relationship.
type FrameRelation struct {
	ParentFrameID string
	ChildFrameID  string
}

// ProcessTFMessage updates parentOf with the frame relationships from
// transforms, then calls onChange with a snapshot copy of the full map.
// It is safe to call concurrently with the same parentOf and mu.
func ProcessTFMessage(
	transforms []FrameRelation,
	parentOf map[string]string,
	mu *sync.Mutex,
	onChange func(snapshot map[string]string),
) {
	mu.Lock()
	for _, tf := range transforms {
		if tf.ParentFrameID != "" && tf.ChildFrameID != "" {
			parentOf[tf.ChildFrameID] = tf.ParentFrameID
		}
	}
	snapshot := make(map[string]string, len(parentOf))
	for k, v := range parentOf {
		snapshot[k] = v
	}
	mu.Unlock()
	onChange(snapshot)
}

// TFSubscriber manages native ROS 2 subscriptions to /tf and /tf_static.
type TFSubscriber struct {
	nodeName string
	parentOf map[string]string
	mu       sync.Mutex
}

// NewTFSubscriber creates a subscriber with the given node name.
func NewTFSubscriber(nodeName string) *TFSubscriber {
	return &TFSubscriber{
		nodeName: nodeName,
		parentOf: make(map[string]string),
	}
}

// Run initializes rclgo, creates the node and subscriptions, and spins
// until ctx is cancelled. Calls onChange when new frame relationships arrive.
func (s *TFSubscriber) Run(ctx context.Context, onChange func(parentOf map[string]string)) error {
	rclArgs, _, err := rclgo.ParseArgs(nil)
	if err != nil {
		return fmt.Errorf("failed to parse ROS args: %w", err)
	}

	if err := rclgo.Init(rclArgs); err != nil {
		return fmt.Errorf("could not initialize ROS 2: %w", err)
	}
	defer rclgo.Uninit()

	node, err := rclgo.NewNode(s.nodeName, "")
	if err != nil {
		return fmt.Errorf("could not create ROS 2 node: %w", err)
	}
	defer node.Close()

	makeCallback := func() func(msg *tf2_msgs_msg.TFMessage, info *rclgo.MessageInfo, err error) {
		return func(msg *tf2_msgs_msg.TFMessage, info *rclgo.MessageInfo, callbackErr error) {
			if callbackErr != nil {
				return
			}
			relations := make([]FrameRelation, 0, len(msg.Transforms))
			for _, tf := range msg.Transforms {
				relations = append(relations, FrameRelation{
					ParentFrameID: tf.Header.FrameId,
					ChildFrameID:  tf.ChildFrameId,
				})
			}
			ProcessTFMessage(relations, s.parentOf, &s.mu, onChange)
		}
	}

	// Subscribe to /tf with default QoS
	tfSub, err := tf2_msgs_msg.NewTFMessageSubscription(
		node, "/tf", nil, makeCallback(),
	)
	if err != nil {
		return fmt.Errorf("could not subscribe to /tf: %w", err)
	}
	defer tfSub.Close()

	// Subscribe to /tf_static with transient local durability
	staticOpts := &rclgo.SubscriptionOptions{
		Qos: rclgo.QosProfile{
			History:     rclgo.HistoryKeepLast,
			Depth:       10,
			Reliability: rclgo.ReliabilityReliable,
			Durability:  rclgo.DurabilityTransientLocal,
		},
	}
	tfStaticSub, err := tf2_msgs_msg.NewTFMessageSubscription(
		node, "/tf_static", staticOpts, makeCallback(),
	)
	if err != nil {
		return fmt.Errorf("could not subscribe to /tf_static: %w", err)
	}
	defer tfStaticSub.Close()

	ws, err := rclgo.NewWaitSet()
	if err != nil {
		return fmt.Errorf("could not create waitset: %w", err)
	}
	defer ws.Close()
	ws.AddSubscriptions(tfSub.Subscription, tfStaticSub.Subscription)

	return ws.Run(ctx)
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source cgo-flags.env && go test -v -race -run 'TestProcessTFMessage' ./internal/tftree/
```

Expected: All 4 tests PASS.

- [ ] **Step 5: Remove the old CollectFromROS2 and echoTopic functions**

Modify `internal/tftree/collector.go`:
- Remove `CollectFromROS2` function (lines 29-71)
- Remove `echoTopic` function (lines 75-109)
- Remove unused imports: `context`, `fmt`, `os/exec`, `strings`, `sync`, `time`
- Keep: `ParseTFYAML`, `parseTFDocument`, `TFMessage` struct (used by --stdin mode)

The file should only contain:
```go
package tftree

import (
	"bufio"
	"bytes"
	"io"

	"gopkg.in/yaml.v3"
)

// TFMessage represents a single ROS 2 TFMessage (tf2_msgs/msg/TFMessage).
// Used for parsing YAML output from `ros2 topic echo`.
type TFMessage struct {
	Transforms []struct {
		Header struct {
			FrameID string `yaml:"frame_id"`
		} `yaml:"header"`
		ChildFrameID string `yaml:"child_frame_id"`
	} `yaml:"transforms"`
}

// ParseTFYAML parses streaming YAML output from `ros2 topic echo /tf` and
// returns a map of child→parent frame relationships. Documents are separated
// by "---" lines.
func ParseTFYAML(r io.Reader) (map[string]string, error) {
	parentOf := make(map[string]string)
	scanner := bufio.NewScanner(r)
	var buf bytes.Buffer

	for scanner.Scan() {
		line := scanner.Text()
		if line == "---" {
			if buf.Len() > 0 {
				parseTFDocument(buf.Bytes(), parentOf)
				buf.Reset()
			}
			continue
		}
		buf.WriteString(line)
		buf.WriteByte('\n')
	}

	// Parse any remaining data.
	if buf.Len() > 0 {
		parseTFDocument(buf.Bytes(), parentOf)
	}

	return parentOf, scanner.Err()
}

func parseTFDocument(data []byte, parentOf map[string]string) {
	var msg TFMessage
	if err := yaml.Unmarshal(data, &msg); err != nil {
		return
	}
	for _, tf := range msg.Transforms {
		parent := tf.Header.FrameID
		child := tf.ChildFrameID
		if parent != "" && child != "" {
			parentOf[child] = parent
		}
	}
}
```

- [ ] **Step 6: Run all existing tests to verify nothing is broken**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source cgo-flags.env && go test -v -race ./internal/tftree/
```

Expected: All tests PASS (existing tree tests + new subscriber tests).

- [ ] **Step 7: Commit**

```bash
git add internal/tftree/subscriber.go internal/tftree/subscriber_test.go internal/tftree/collector.go
git commit -m "feat: add native ROS 2 TF subscriber via rclgo

Implement TFSubscriber that creates a ROS 2 node and subscribes to
/tf and /tf_static topics using PolibaX/rclgo. Factor out
ProcessTFMessage for testable frame extraction logic.

Remove old CollectFromROS2 subprocess approach. Keep ParseTFYAML
for --stdin fallback mode.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Chunk 3: Updated main.go with Live Mode & Debounce

### Task 3: Rewrite main.go for live mode with debounced rendering

**Files:**
- Modify: `main.go`

- [ ] **Step 1: Rewrite main.go**

Replace the contents of `main.go` with:

```go
// tftree-cli is a terminal-based ROS 2 TF tree visualization tool.
//
// It subscribes to /tf and /tf_static topics using native ROS 2 bindings,
// collects transform data, and displays the frame hierarchy as a live-updating
// tree using Unicode box-drawing characters.
//
// Usage:
//
//	tftree-cli [flags] [output-file]
//	ros2 topic echo /tf | tftree-cli --stdin
package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"sync"
	"time"

	"github.com/Maverobot/tftree-cli/internal/tftree"
	"golang.org/x/term"
)

var version = "dev"

func main() {
	var (
		useStdin    = flag.Bool("stdin", false, "Read TF YAML data from stdin instead of subscribing to ROS 2 topics")
		refresh     = flag.Duration("refresh", 500*time.Millisecond, "Debounce interval for live display refresh")
		nodeName    = flag.String("node-name", "tftree_cli", "ROS 2 node name")
		showVersion = flag.Bool("version", false, "Print version and exit")
	)

	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "tftree-cli - Terminal-based ROS 2 TF tree viewer\n\n")
		fmt.Fprintf(os.Stderr, "Usage:\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli [flags] [output-file]\n\n")
		fmt.Fprintf(os.Stderr, "Examples:\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli                          # Live mode, subscribe to /tf\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli -refresh 1s              # Refresh every 1s\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli tree.txt                 # Save final snapshot to file on exit\n")
		fmt.Fprintf(os.Stderr, "  ros2 topic echo /tf | tftree-cli --stdin\n\n")
		fmt.Fprintf(os.Stderr, "Flags:\n")
		flag.PrintDefaults()
	}

	flag.Parse()

	if *showVersion {
		fmt.Printf("tftree-cli %s\n", version)
		os.Exit(0)
	}

	var outputFile string
	if flag.NArg() > 0 {
		outputFile = flag.Arg(0)
	}

	if *useStdin {
		runStdinMode(outputFile)
	} else {
		runLiveMode(*nodeName, *refresh, outputFile)
	}
}

func runStdinMode(outputFile string) {
	fmt.Fprintln(os.Stderr, "Reading TF data from stdin (press Ctrl+C or Ctrl+D when done)...")
	parentOf, err := tftree.ParseTFYAML(os.Stdin)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		os.Exit(1)
	}

	tree := tftree.NewTree(parentOf)
	tree.Render(os.Stdout)

	if outputFile != "" {
		saveToFile(tree, outputFile)
	}
}

func runLiveMode(nodeName string, refresh time.Duration, outputFile string) {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	isTTY := term.IsTerminal(int(os.Stdout.Fd()))

	var (
		latestParentOf map[string]string
		mu             sync.Mutex
		timer          *time.Timer
		rendered       bool
	)

	sub := tftree.NewTFSubscriber(nodeName)

	fmt.Fprintln(os.Stderr, "Subscribing to /tf and /tf_static... (press Ctrl+C to exit)")

	onChange := func(parentOf map[string]string) {
		mu.Lock()
		latestParentOf = parentOf
		if timer != nil {
			timer.Reset(refresh)
		} else {
			timer = time.AfterFunc(refresh, func() {
				mu.Lock()
				snapshot := latestParentOf
				mu.Unlock()

				if snapshot == nil {
					return
				}

				tree := tftree.NewTree(snapshot)
				if isTTY {
					fmt.Print("\033[2J\033[H")
				}
				tree.Render(os.Stdout)

				// Non-TTY: print once and exit
				if !isTTY {
					mu.Lock()
					rendered = true
					mu.Unlock()
					stop()
				}
			})
		}
		mu.Unlock()
	}

	err := sub.Run(ctx, onChange)
	if err != nil && ctx.Err() == nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		fmt.Fprintln(os.Stderr, "Ensure ROS 2 Jazzy is installed and sourced. You can also use --stdin mode.")
		os.Exit(1)
	}

	// Print final snapshot on exit (skip if already rendered in non-TTY mode)
	mu.Lock()
	finalParentOf := latestParentOf
	alreadyRendered := rendered
	mu.Unlock()

	if finalParentOf != nil && !alreadyRendered {
		tree := tftree.NewTree(finalParentOf)
		if isTTY {
			fmt.Print("\033[2J\033[H")
		}
		tree.Render(os.Stdout)

		if outputFile != "" {
			saveToFile(tree, outputFile)
		}
	} else if finalParentOf != nil && outputFile != "" {
		tree := tftree.NewTree(finalParentOf)
		saveToFile(tree, outputFile)
	}
}

func saveToFile(tree *tftree.Tree, outputFile string) {
	output := tree.RenderString()
	if err := os.WriteFile(outputFile, []byte(output), 0644); err != nil {
		fmt.Fprintf(os.Stderr, "Error: could not save to file: %v\n", err)
		os.Exit(1)
	}
	absPath, _ := filepath.Abs(outputFile)
	fmt.Fprintf(os.Stderr, "Tree saved to: %s\n", absPath)
}
```

- [ ] **Step 2: Add golang.org/x/term dependency**

Run:
```bash
go get golang.org/x/term
go mod tidy
```

Expected: `golang.org/x/term` added to `go.mod`.

- [ ] **Step 3: Verify the build compiles**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source cgo-flags.env && go build -v .
```

Expected: Binary built successfully.

- [ ] **Step 4: Run all tests**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source cgo-flags.env && go test -v -race ./...
```

Expected: All tests PASS.

- [ ] **Step 5: Verify go vet passes**

Run:
```bash
source /opt/ros/jazzy/setup.bash && source cgo-flags.env && go vet ./...
```

Expected: No issues.

- [ ] **Step 6: Commit**

```bash
git add main.go go.mod go.sum
git commit -m "feat: live mode with debounced rendering and TTY detection

Rewrite main.go to use TFSubscriber for continuous live display.
Add debounced rendering with configurable --refresh interval.
Detect TTY for ANSI clear codes. Keep --stdin as fallback.
Remove --duration flag (replaced by continuous mode).

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Chunk 4: AppImage CI Pipeline

### Task 4: Create the AppImage GitHub Actions workflow

**Files:**
- Create: `.github/workflows/appimage.yml`
- Create: `packaging/tftree-cli.desktop`
- Create: `packaging/AppRun`

- [ ] **Step 1: Create the desktop entry file**

Create `packaging/tftree-cli.desktop`:
```ini
[Desktop Entry]
Type=Application
Name=tftree-cli
Comment=Terminal-based ROS 2 TF tree viewer
Exec=tftree-cli
Icon=tftree-cli
Terminal=true
Categories=Development;RoboticsEngineering;
```

- [ ] **Step 2: Create the AppRun script**

Create `packaging/AppRun`:
```bash
#!/bin/bash
HERE="$(dirname "$(readlink -f "${0}")")"
export LD_LIBRARY_PATH="${HERE}/usr/lib:${LD_LIBRARY_PATH}"
export AMENT_PREFIX_PATH="${HERE}/usr:${AMENT_PREFIX_PATH}"
exec "${HERE}/usr/bin/tftree-cli" "$@"
```

Make it executable:
```bash
chmod +x packaging/AppRun
```

- [ ] **Step 3: Create a placeholder icon**

Create a simple 1x1 PNG as a placeholder icon (AppImage requires an icon):

```bash
python3 -c "
import struct, zlib
def create_png():
    sig = b'\x89PNG\r\n\x1a\n'
    ihdr_data = struct.pack('>IIBBBBB', 1, 1, 8, 2, 0, 0, 0)
    ihdr_crc = zlib.crc32(b'IHDR' + ihdr_data) & 0xffffffff
    ihdr = struct.pack('>I', 13) + b'IHDR' + ihdr_data + struct.pack('>I', ihdr_crc)
    raw = b'\x00\x00\x00\x00'
    idat_data = zlib.compress(raw)
    idat_crc = zlib.crc32(b'IDAT' + idat_data) & 0xffffffff
    idat = struct.pack('>I', len(idat_data)) + b'IDAT' + idat_data + struct.pack('>I', idat_crc)
    iend_crc = zlib.crc32(b'IEND') & 0xffffffff
    iend = struct.pack('>I', 0) + b'IEND' + struct.pack('>I', iend_crc)
    return sig + ihdr + idat + iend
with open('packaging/tftree-cli.png', 'wb') as f:
    f.write(create_png())
" 
```

- [ ] **Step 4: Create the AppImage workflow**

Create `.github/workflows/appimage.yml`:
```yaml
name: AppImage

on:
  push:
    branches: [main]
    tags: ["v*"]

permissions:
  contents: write

jobs:
  appimage:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-go@v5
        with:
          go-version-file: go.mod

      - name: Install ROS 2 Jazzy
        run: |
          sudo apt-get update
          sudo apt-get install -y software-properties-common curl
          sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
          echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
          sudo apt-get update
          sudo apt-get install -y ros-jazzy-ros-core ros-jazzy-tf2-msgs

      - name: Generate message bindings
        run: |
          source /opt/ros/jazzy/setup.bash
          go generate ./...

      - name: Build binary
        run: |
          source /opt/ros/jazzy/setup.bash
          source cgo-flags.env
          CGO_ENABLED=1 go build -v -ldflags "-s -w -X main.version=${{ github.ref_name }}" -o tftree-cli .

      - name: Prepare AppDir
        run: |
          mkdir -p AppDir/usr/bin AppDir/usr/lib
          cp tftree-cli AppDir/usr/bin/
          cp packaging/AppRun AppDir/
          cp packaging/tftree-cli.desktop AppDir/
          cp packaging/tftree-cli.png AppDir/

          # Bundle ROS 2 shared libraries
          for lib in $(ldd AppDir/usr/bin/tftree-cli | grep '/opt/ros\|librcl\|librmw\|librosidl\|librcutils\|libfastcdr\|libfastdds\|libfoonathan\|libtinyxml2\|libspdlog\|libfmt\|libyaml\|libament' | awk '{print $3}'); do
            cp -L "$lib" AppDir/usr/lib/ 2>/dev/null || true
          done

          # Also bundle any transitive deps from /opt/ros
          for lib in AppDir/usr/lib/*.so*; do
            for dep in $(ldd "$lib" 2>/dev/null | grep '/opt/ros' | awk '{print $3}'); do
              cp -L "$dep" AppDir/usr/lib/ 2>/dev/null || true
            done
          done

      - name: Download appimagetool
        run: |
          curl -sSL -o appimagetool https://github.com/AppImage/appimagetool/releases/download/continuous/appimagetool-x86_64.AppImage
          chmod +x appimagetool

      - name: Build AppImage
        run: |
          ARCH=x86_64 ./appimagetool --no-appstream AppDir tftree-cli-x86_64.AppImage

      - name: Upload AppImage artifact
        uses: actions/upload-artifact@v4
        with:
          name: tftree-cli-appimage
          path: tftree-cli-x86_64.AppImage

      - name: Upload to release
        if: startsWith(github.ref, 'refs/tags/')
        uses: softprops/action-gh-release@v2
        with:
          files: tftree-cli-x86_64.AppImage
```

- [ ] **Step 5: Update .goreleaser.yml to exclude AppImage (handled by separate workflow)**

No changes needed — GoReleaser handles tar.gz/zip/deb/rpm/apk. AppImage is built by the new workflow.

- [ ] **Step 6: Commit**

```bash
git add .github/workflows/appimage.yml packaging/
git commit -m "ci: add AppImage build and release workflow

Build self-contained AppImage bundling ROS 2 Jazzy libraries on
every push to main and on tag releases. Uses appimagetool to
package the binary with all required shared libraries.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

---

## Chunk 5: Update CI, Documentation & Cleanup

### Task 5: Update CI workflow for CGO builds

**Files:**
- Modify: `.github/workflows/ci.yml`

- [ ] **Step 1: Update CI to install ROS 2 and use CGO**

Replace `.github/workflows/ci.yml`:
```yaml
name: CI

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

permissions:
  contents: read

jobs:
  test:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-go@v5
        with:
          go-version-file: go.mod

      - name: Install ROS 2 Jazzy
        run: |
          sudo apt-get update
          sudo apt-get install -y software-properties-common curl
          sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
          echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
          sudo apt-get update
          sudo apt-get install -y ros-jazzy-ros-core ros-jazzy-tf2-msgs

      - name: Generate message bindings
        run: |
          source /opt/ros/jazzy/setup.bash
          go generate ./...

      - name: Build
        run: |
          source /opt/ros/jazzy/setup.bash
          source cgo-flags.env
          CGO_ENABLED=1 go build -v ./...

      - name: Test
        run: |
          source /opt/ros/jazzy/setup.bash
          source cgo-flags.env
          CGO_ENABLED=1 go test -v -race ./...

      - name: Vet
        run: |
          source /opt/ros/jazzy/setup.bash
          source cgo-flags.env
          CGO_ENABLED=1 go vet ./...
```

- [ ] **Step 2: Commit**

```bash
git add .github/workflows/ci.yml
git commit -m "ci: update CI to install ROS 2 Jazzy for CGO builds

CI now installs ros-jazzy-ros-core and ros-jazzy-tf2-msgs,
generates message bindings, and builds/tests with CGO enabled.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

### Task 6: Update README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Update README for new functionality**

Update `README.md` to reflect:
- Live mode is now the default (native ROS 2 subscription)
- New flags: `--refresh`, `--node-name`
- Removed flags: `--duration`
- AppImage distribution
- Build requirements (ROS 2 Jazzy for building from source)

Key sections to update:
- Features: add "Native ROS 2 subscription" and "Live updating display"
- Installation: add AppImage download option
- Usage: update live mode section, remove duration references
- How It Works: update to describe native subscription
- Building from Source: add ROS 2 Jazzy prerequisite

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: update README for native subscription and AppImage

Update documentation to reflect live mode with native ROS 2
subscriptions, new CLI flags, AppImage distribution, and
build requirements.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```

### Task 7: Update GoReleaser for CGO builds

**Files:**
- Modify: `.goreleaser.yml`

- [ ] **Step 1: Update .goreleaser.yml**

Since the binary now requires CGO (rclgo links against ROS 2 C libraries), the GoReleaser config needs updating. For non-AppImage releases, we only build Linux amd64 with CGO enabled (the primary target). Other platforms would need their own ROS 2 installations.

Update `.goreleaser.yml`:
```yaml
version: 2

project_name: tftree-cli

builds:
  - main: .
    binary: tftree-cli
    env:
      - CGO_ENABLED=1
    goos:
      - linux
    goarch:
      - amd64
    ldflags:
      - -s -w -X main.version={{.Version}}

archives:
  - format: tar.gz
    name_template: "{{ .ProjectName }}_{{ .Version }}_{{ .Os }}_{{ .Arch }}"

nfpms:
  - package_name: tftree-cli
    homepage: https://github.com/Maverobot/tftree-cli
    description: Terminal-based ROS 2 TF tree viewer
    license: MIT
    formats:
      - deb
      - rpm

checksum:
  name_template: checksums.txt

changelog:
  sort: asc
  filters:
    exclude:
      - "^docs:"
      - "^test:"
      - "^ci:"
```

- [ ] **Step 2: Update release workflow for ROS 2**

Modify `.github/workflows/release.yml`:
```yaml
name: Release

on:
  push:
    tags:
      - "v*"

permissions:
  contents: write

jobs:
  release:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0

      - uses: actions/setup-go@v5
        with:
          go-version-file: go.mod

      - name: Install ROS 2 Jazzy
        run: |
          sudo apt-get update
          sudo apt-get install -y software-properties-common curl
          sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
          echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
          sudo apt-get update
          sudo apt-get install -y ros-jazzy-ros-core ros-jazzy-tf2-msgs

      - name: Generate message bindings
        run: |
          source /opt/ros/jazzy/setup.bash
          go generate ./...

      - name: Export CGO flags
        run: |
          source cgo-flags.env
          echo "CGO_CFLAGS=${CGO_CFLAGS}" >> $GITHUB_ENV
          echo "CGO_LDFLAGS=${CGO_LDFLAGS}" >> $GITHUB_ENV

      - name: Run GoReleaser
        uses: goreleaser/goreleaser-action@v6
        with:
          version: "~> v2"
          args: release --clean
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          CGO_ENABLED: "1"
```

- [ ] **Step 3: Commit**

```bash
git add .goreleaser.yml .github/workflows/release.yml
git commit -m "ci: update GoReleaser and release workflow for CGO builds

Limit GoReleaser to Linux amd64 (CGO requirement). Update release
workflow to install ROS 2 Jazzy and generate message bindings
before building.

Co-authored-by: Copilot <223556219+Copilot@users.noreply.github.com>"
```
