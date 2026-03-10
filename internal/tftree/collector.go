package tftree

import (
	"bufio"
	"bytes"
	"io"

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
