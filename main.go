// tftree-cli is a terminal-based ROS 2 TF tree visualization tool.
//
// It subscribes to /tf and /tf_static topics, collects transform data,
// and displays the frame hierarchy as a tree using Unicode box-drawing
// characters. Inspired by tf_tree_terminal.
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
	"time"

	"github.com/Maverobot/tftree-cli/internal/tftree"
)

var version = "dev"

func main() {
	var (
		duration    = flag.Duration("duration", 2*time.Second, "Duration to listen for TF data")
		useStdin    = flag.Bool("stdin", false, "Read TF YAML data from stdin instead of subscribing to ROS 2 topics")
		showVersion = flag.Bool("version", false, "Print version and exit")
	)

	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "tftree-cli - Terminal-based ROS 2 TF tree viewer\n\n")
		fmt.Fprintf(os.Stderr, "Usage:\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli [flags] [output-file]\n\n")
		fmt.Fprintf(os.Stderr, "Examples:\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli                          # Listen for 2s, print tree\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli -duration 5s             # Listen for 5s\n")
		fmt.Fprintf(os.Stderr, "  tftree-cli tree.txt                 # Save output to file\n")
		fmt.Fprintf(os.Stderr, "  ros2 topic echo /tf | tftree-cli --stdin\n\n")
		fmt.Fprintf(os.Stderr, "Flags:\n")
		flag.PrintDefaults()
	}

	flag.Parse()

	if *showVersion {
		fmt.Printf("tftree-cli %s\n", version)
		os.Exit(0)
	}

	// Determine output file from positional arguments.
	var outputFile string
	if flag.NArg() > 0 {
		outputFile = flag.Arg(0)
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt)
	defer stop()

	var parentOf map[string]string
	var err error

	if *useStdin {
		fmt.Fprintln(os.Stderr, "Reading TF data from stdin (press Ctrl+C or Ctrl+D when done)...")
		parentOf, err = tftree.ParseTFYAML(os.Stdin)
	} else {
		fmt.Fprintf(os.Stderr, "Buffering TF data for %s...\n", *duration)
		parentOf, err = tftree.CollectFromROS2(ctx, *duration)
	}

	if err != nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		os.Exit(1)
	}

	tree := tftree.NewTree(parentOf)

	// Print to stdout.
	tree.Render(os.Stdout)

	// Save to file if requested.
	if outputFile != "" {
		output := tree.RenderString()
		if err := os.WriteFile(outputFile, []byte(output), 0644); err != nil {
			fmt.Fprintf(os.Stderr, "Error: could not save to file: %v\n", err)
			os.Exit(1)
		}
		absPath, _ := filepath.Abs(outputFile)
		fmt.Fprintf(os.Stderr, "Tree saved to: %s\n", absPath)
	}
}
