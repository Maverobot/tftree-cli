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
