# Native TF Subscription & AppImage Release

## Problem Statement

tftree-cli currently shells out to `ros2 topic echo` to collect TF data, which is fragile, requires the ROS 2 CLI to be installed, and only supports a one-shot duration-based collection mode. Users want:

1. Native ROS 2 topic subscription for direct `/tf` and `/tf_static` access
2. Continuous live display that updates in real-time
3. Self-contained AppImage distribution that bundles ROS 2 libraries

## Approach

Use the PolibaX/rclgo fork (Jazzy-compatible Go bindings for ROS 2) to create a native subscriber node. Replace the subprocess-based collector with a proper ROS 2 subscription. Keep `--stdin` as a fallback mode. Package as AppImage via GitHub Actions CI.

## Architecture

### Package Structure

```
main.go                          # CLI entry point, flag parsing
internal/tftree/
  tree.go                        # Tree building & rendering (unchanged)
  tree_test.go                   # Tree tests (unchanged)
  collector.go                   # ParseTFYAML for --stdin mode (keep)
  subscriber.go                  # Native ROS 2 subscription via rclgo
  subscriber_test.go             # Tests for subscriber
msgs/                            # Generated Go bindings for tf2_msgs
  tf2_msgs/
  geometry_msgs/
  std_msgs/
  builtin_interfaces/
```

### Data Flow

1. **Native mode (default)**: rclgo subscriber → callbacks populate shared `parentOf` map → debounced timer triggers → clear screen + `Tree.Render()`
2. **Stdin mode (`--stdin`)**: Same as today — `ParseTFYAML` → `NewTree` → `Render`

The `CollectFromROS2` function (which shells out to `ros2 topic echo`) is removed.

## Native ROS 2 Subscription

### rclgo Fork

Use `github.com/PolibaX/rclgo` — a fork of tiiuae/rclgo with ROS 2 Jazzy support.

### Subscriber Interface

```go
// TFSubscriber manages native ROS 2 subscriptions to /tf and /tf_static.
type TFSubscriber struct { ... }

// NewTFSubscriber creates a subscriber with the given node name.
// Returns an error if ROS 2 middleware initialization fails.
func NewTFSubscriber(nodeName string) (*TFSubscriber, error)

// Start begins subscribing. Calls onChange when new frame relationships arrive.
// The onChange callback receives the updated child→parent map (a snapshot copy).
// Blocks until ctx is cancelled.
func (s *TFSubscriber) Start(ctx context.Context, onChange func(parentOf map[string]string)) error

// Close destroys the ROS 2 node and frees resources.
func (s *TFSubscriber) Close()
```

The subscriber owns the `parentOf` map internally. On each TF message callback, it updates the map under a mutex and calls `onChange` with a snapshot copy of the full map. This keeps the map ownership clear and thread-safe.

### Subscriber Behavior

- Create a ROS 2 node named `tftree_cli` (configurable via `--node-name`)
- Subscribe to `/tf` with default QoS
- Subscribe to `/tf_static` with transient local QoS (for latched static transforms)
- On each message callback: extract `frame_id` → `child_frame_id` pairs, update internal `parentOf` map, call `onChange` with a snapshot

### Error Handling

- **ROS 2 middleware unavailable** (e.g., no DDS, no ROS 2 libs): `NewTFSubscriber` returns an error. `main.go` prints a clear message: `"Error: could not initialize ROS 2. Ensure ROS 2 Jazzy is installed and sourced. You can also use --stdin mode."` and exits with code 1.
- **Topic not publishing**: The subscriber simply waits. No timeout — the user can Ctrl+C when they're done. The display shows `"Waiting for TF data..."` until the first message arrives.
- **Middleware crashes mid-run**: `Start` returns an error. `main.go` prints the error and exits.

### Debounced Rendering

Debounce logic lives in `main.go`, not in the subscriber. The `onChange` callback resets a debounce timer.

- Default debounce interval: 500ms (configurable via `--refresh` flag)
- After the first TF message arrives, start a timer; each new message resets it
- When the timer fires: if stdout is a TTY, clear terminal (`\033[2J\033[H`) then call `Tree.Render()`; if stdout is not a TTY (piped/redirected), skip clearing and print normally (one-shot behavior — print once and exit after first debounce)
- This keeps CPU usage low even with 100+ Hz TF publishers

### Graceful Shutdown

On `SIGINT`/`SIGTERM`:
1. Cancel the subscription context
2. Destroy the ROS 2 node
3. Print the final tree rendering to stdout (without ANSI clear codes)
4. If an output file was specified, save the final rendering (same format as stdout — the Unicode tree output)

## CLI Interface

```
tftree-cli [flags] [output-file]

Flags:
  --stdin            Read TF YAML from stdin (one-shot mode)
  --refresh 500ms    Debounce interval for live display (default 500ms)
  --node-name NAME   ROS 2 node name (default "tftree_cli")
  --version          Print version and exit
```

### Removed Flags

- `--duration` — no longer needed; live mode runs until Ctrl+C

### Modes

1. `tftree-cli` → Live mode: subscribe, continuously display, Ctrl+C to exit
2. `tftree-cli --stdin` → One-shot: read from stdin, print tree, exit
3. `tftree-cli tree.txt` → Live mode with file save on exit

### Flag Conflicts

- `--stdin` with `--refresh` or `--node-name`: the ROS-specific flags are silently ignored in stdin mode (stdin is always one-shot)
- Passing an output file with `--stdin`: save the tree to the file after stdin is consumed (same as current behavior)

## AppImage CI Pipeline

### Workflow File

`.github/workflows/appimage.yml`

### Triggers

- Push to `main` branch
- Tag pushes (`v*`)

### Build Steps

1. Use `ubuntu-24.04` runner
2. Install ROS 2 Jazzy (`ros-jazzy-ros-core`, `ros-jazzy-tf2-msgs`) from official apt repo
3. Install Go via `actions/setup-go`
4. Generate rclgo message bindings with `rclgo-gen`
5. Build binary with CGO enabled, linking against ROS 2 libs
6. Package as AppImage using `linuxdeploy` — bundle all required `.so` files
7. Upload as release artifact (for tags) or workflow artifact (for main pushes)

### AppImage Contents

```
tftree-cli.AppImage
├── AppRun                    # Launcher script (sets LD_LIBRARY_PATH)
├── tftree-cli.desktop        # Desktop entry
├── tftree-cli.png            # Icon (placeholder)
├── usr/bin/tftree-cli        # Binary
└── usr/lib/                  # Bundled ROS 2 shared libraries
```

## Testing Strategy

1. **Tree rendering tests** — existing tests unchanged
2. **ParseTFYAML tests** — existing tests unchanged
3. **Subscriber unit tests** — test the `onChange` callback logic:
   - Create a `TFSubscriber`, simulate TF message callbacks by directly calling the internal message handler
   - Verify that the `onChange` callback receives correct `parentOf` map snapshots
   - Verify thread safety: concurrent callback invocations produce consistent maps
   - Mock boundary: the rclgo `Subscription` type is wrapped, so tests bypass rclgo entirely and test the transform extraction + map management logic
4. **Debounce tests** — test the debounce timer in `main.go`:
   - Rapid onChange calls result in only one render after the debounce interval
5. **Build verification** — CI builds the binary as part of AppImage workflow (compilation test)
6. **Manual integration tests** — run with a live ROS 2 system to verify end-to-end behavior
