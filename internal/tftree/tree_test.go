package tftree

import (
	"strings"
	"testing"
)

func TestNewTreeAndRoots(t *testing.T) {
	parentOf := map[string]string{
		"base_link":    "odom",
		"camera_link":  "base_link",
		"chassis_link": "base_link",
		"odom":         "map",
	}

	tree := NewTree(parentOf)
	roots := tree.Roots()

	if len(roots) != 1 {
		t.Fatalf("expected 1 root, got %d: %v", len(roots), roots)
	}
	if roots[0] != "map" {
		t.Errorf("expected root 'map', got %q", roots[0])
	}
}

func TestNewTreeMultipleRoots(t *testing.T) {
	parentOf := map[string]string{
		"child_a": "root_a",
		"child_b": "root_b",
	}

	tree := NewTree(parentOf)
	roots := tree.Roots()

	if len(roots) != 2 {
		t.Fatalf("expected 2 roots, got %d: %v", len(roots), roots)
	}
	if roots[0] != "root_a" || roots[1] != "root_b" {
		t.Errorf("expected roots [root_a, root_b], got %v", roots)
	}
}

func TestTreeRenderEmpty(t *testing.T) {
	tree := NewTree(map[string]string{})
	output := tree.RenderString()

	expected := "No TF tree detected.\n"
	if output != expected {
		t.Errorf("expected %q, got %q", expected, output)
	}
}

func TestTreeRender(t *testing.T) {
	parentOf := map[string]string{
		"odom":           "map",
		"base_footprint": "odom",
		"base_link":      "base_footprint",
		"camera_link":    "base_link",
		"chassis_link":   "base_link",
		"front_axle":     "chassis_link",
		"rear_axle":      "chassis_link",
		"velodyne_link":  "base_link",
	}

	tree := NewTree(parentOf)
	output := tree.RenderString()

	// Verify the output matches the expected tree structure.
	expected := `--- ROS 2 TF Tree Snapshot ---
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
`
	if output != expected {
		t.Errorf("tree output mismatch.\nExpected:\n%s\nGot:\n%s", expected, output)
	}
}

func TestTreeRenderSingleNode(t *testing.T) {
	parentOf := map[string]string{
		"child": "root",
	}

	tree := NewTree(parentOf)
	output := tree.RenderString()

	expected := `--- ROS 2 TF Tree Snapshot ---
root
└── child
------------------------------
`
	if output != expected {
		t.Errorf("expected:\n%s\ngot:\n%s", expected, output)
	}
}

func TestParseTFYAML(t *testing.T) {
	yamlData := `transforms:
- header:
    stamp:
      sec: 1709123456
      nanosec: 789012345
    frame_id: odom
  child_frame_id: base_link
  transform:
    translation:
      x: 1.0
      y: 2.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
- header:
    stamp:
      sec: 1709123456
      nanosec: 789012345
    frame_id: base_link
  child_frame_id: camera_link
  transform:
    translation:
      x: 0.5
      y: 0.0
      z: 0.3
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
transforms:
- header:
    stamp:
      sec: 1709123457
      nanosec: 0
    frame_id: map
  child_frame_id: odom
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 0.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
---
`

	parentOf, err := ParseTFYAML(strings.NewReader(yamlData))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	expected := map[string]string{
		"base_link":   "odom",
		"camera_link": "base_link",
		"odom":        "map",
	}

	if len(parentOf) != len(expected) {
		t.Fatalf("expected %d relations, got %d: %v", len(expected), len(parentOf), parentOf)
	}

	for child, parent := range expected {
		if parentOf[child] != parent {
			t.Errorf("expected parent of %q to be %q, got %q", child, parent, parentOf[child])
		}
	}
}

func TestParseTFYAMLEmpty(t *testing.T) {
	parentOf, err := ParseTFYAML(strings.NewReader(""))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(parentOf) != 0 {
		t.Errorf("expected empty map, got %v", parentOf)
	}
}

func TestParseTFYAMLInvalid(t *testing.T) {
	// Invalid YAML should not cause an error, just be skipped.
	parentOf, err := ParseTFYAML(strings.NewReader("not valid yaml: [[["))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if len(parentOf) != 0 {
		t.Errorf("expected empty map, got %v", parentOf)
	}
}

func TestEndToEnd(t *testing.T) {
	// Test the full pipeline: parse YAML → build tree → render.
	yamlData := `transforms:
- header:
    frame_id: map
  child_frame_id: odom
- header:
    frame_id: odom
  child_frame_id: base_link
- header:
    frame_id: base_link
  child_frame_id: sensor
---
`
	parentOf, err := ParseTFYAML(strings.NewReader(yamlData))
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	tree := NewTree(parentOf)
	output := tree.RenderString()

	expected := `--- ROS 2 TF Tree Snapshot ---
map
└── odom
    └── base_link
        └── sensor
------------------------------
`
	if output != expected {
		t.Errorf("expected:\n%s\ngot:\n%s", expected, output)
	}
}
