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
	"syscall"
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
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	isTTY := term.IsTerminal(int(os.Stdout.Fd()))

	var (
		latestParentOf map[string]string
		mu             sync.Mutex
		dirty          bool
	)

	sub := tftree.NewTFSubscriber(nodeName)

	fmt.Fprintln(os.Stderr, "Subscribing to /tf and /tf_static... (press Ctrl+C to exit)")

	onChange := func(parentOf map[string]string) {
		mu.Lock()
		latestParentOf = parentOf
		dirty = true
		mu.Unlock()
	}

	// Render loop: check for new data at fixed intervals
	go func() {
		ticker := time.NewTicker(refresh)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				mu.Lock()
				if !dirty {
					mu.Unlock()
					continue
				}
				snapshot := latestParentOf
				dirty = false
				mu.Unlock()

				tree := tftree.NewTree(snapshot)
				if isTTY {
					fmt.Print("\033[2J\033[H")
				}
				tree.Render(os.Stdout)

				if !isTTY {
					stop()
					return
				}
			}
		}
	}()

	err := sub.Run(ctx, onChange)
	if err != nil && ctx.Err() == nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		fmt.Fprintln(os.Stderr, "Ensure ROS 2 Jazzy is installed and sourced. You can also use --stdin mode.")
		os.Exit(1)
	}

	// Final render on exit
	mu.Lock()
	finalParentOf := latestParentOf
	mu.Unlock()

	if finalParentOf != nil {
		tree := tftree.NewTree(finalParentOf)
		if isTTY {
			fmt.Print("\033[2J\033[H")
		}
		tree.Render(os.Stdout)

		if outputFile != "" {
			saveToFile(tree, outputFile)
		}
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
