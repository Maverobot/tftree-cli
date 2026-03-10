package tftree

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"io"
	"os/exec"
	"strings"
	"sync"
	"time"

	"gopkg.in/yaml.v3"
)

// TFMessage represents a single ROS 2 TFMessage (tf2_msgs/msg/TFMessage).
type TFMessage struct {
	Transforms []struct {
		Header struct {
			FrameID string `yaml:"frame_id"`
		} `yaml:"header"`
		ChildFrameID string `yaml:"child_frame_id"`
	} `yaml:"transforms"`
}

// CollectFromROS2 subscribes to /tf and /tf_static topics for the given
// duration and returns a map of child→parent frame relationships.
func CollectFromROS2(ctx context.Context, duration time.Duration) (map[string]string, error) {
	ctx, cancel := context.WithTimeout(ctx, duration+2*time.Second)
	defer cancel()

	parentOf := make(map[string]string)
	var mu sync.Mutex
	var wg sync.WaitGroup
	errCh := make(chan error, 2)

	for _, topic := range []string{"/tf", "/tf_static"} {
		wg.Add(1)
		go func(topic string) {
			defer wg.Done()
			relations, err := echoTopic(ctx, topic, duration)
			if err != nil {
				errCh <- fmt.Errorf("topic %s: %w", topic, err)
				return
			}
			mu.Lock()
			for child, parent := range relations {
				parentOf[child] = parent
			}
			mu.Unlock()
		}(topic)
	}

	wg.Wait()
	close(errCh)

	// Return results even if one topic failed (e.g. /tf_static might not exist).
	if len(parentOf) == 0 {
		// Collect errors only if we got no data at all.
		var errs []string
		for err := range errCh {
			errs = append(errs, err.Error())
		}
		if len(errs) > 0 {
			return nil, fmt.Errorf("failed to collect TF data: %s", strings.Join(errs, "; "))
		}
	}

	return parentOf, nil
}

// echoTopic runs `ros2 topic echo <topic>` for the given duration and parses
// the YAML output into parent→child frame relationships.
func echoTopic(ctx context.Context, topic string, duration time.Duration) (map[string]string, error) {
	cmd := exec.CommandContext(ctx, "ros2", "topic", "echo", topic)
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return nil, fmt.Errorf("creating stdout pipe: %w", err)
	}

	if err := cmd.Start(); err != nil {
		return nil, fmt.Errorf("starting ros2 command: %w", err)
	}

	parentOf := make(map[string]string)
	done := make(chan struct{})

	go func() {
		defer close(done)
		relations, _ := ParseTFYAML(stdout)
		for child, parent := range relations {
			parentOf[child] = parent
		}
	}()

	// Wait for the collection duration, then kill the process.
	select {
	case <-time.After(duration):
	case <-ctx.Done():
	}

	// Kill the process; ignore errors since context cancellation may have already done it.
	_ = cmd.Process.Kill()
	<-done
	_ = cmd.Wait()

	return parentOf, nil
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
