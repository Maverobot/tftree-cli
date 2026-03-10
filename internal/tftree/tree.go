package tftree

import (
	"fmt"
	"io"
	"sort"
	"strings"
)

// Tree represents a TF frame tree built from parent→child relationships.
type Tree struct {
	// children maps parent frame ID to a sorted list of child frame IDs.
	children map[string][]string
}

// NewTree creates a new Tree from a set of parent→child relationships.
// Each key in the map is a child frame ID, and the value is its parent frame ID.
func NewTree(parentOf map[string]string) *Tree {
	children := make(map[string][]string)
	for child, parent := range parentOf {
		children[parent] = append(children[parent], child)
	}
	// Sort children for deterministic output.
	for _, kids := range children {
		sort.Strings(kids)
	}
	return &Tree{children: children}
}

// Roots returns the root frames (parents that are not children of any frame).
func (t *Tree) Roots() []string {
	childSet := make(map[string]bool)
	for _, kids := range t.children {
		for _, kid := range kids {
			childSet[kid] = true
		}
	}
	var roots []string
	for parent := range t.children {
		if !childSet[parent] {
			roots = append(roots, parent)
		}
	}
	sort.Strings(roots)
	return roots
}

// Render writes the tree to the given writer using Unicode box-drawing characters.
func (t *Tree) Render(w io.Writer) {
	roots := t.Roots()
	if len(roots) == 0 {
		fmt.Fprintln(w, "No TF tree detected.")
		return
	}

	fmt.Fprintln(w, "--- ROS 2 TF Tree Snapshot ---")
	for _, root := range roots {
		t.renderNode(w, root, "", true, true)
	}
	fmt.Fprintln(w, "------------------------------")
}

// RenderString returns the tree as a formatted string.
func (t *Tree) RenderString() string {
	var sb strings.Builder
	t.Render(&sb)
	return sb.String()
}

func (t *Tree) renderNode(w io.Writer, frame, indent string, isLast, isRoot bool) {
	marker := ""
	if !isRoot {
		if isLast {
			marker = "└── "
		} else {
			marker = "├── "
		}
	}
	fmt.Fprintf(w, "%s%s%s\n", indent, marker, frame)

	newIndent := indent
	if !isRoot {
		if isLast {
			newIndent += "    "
		} else {
			newIndent += "│   "
		}
	}

	kids := t.children[frame]
	for i, child := range kids {
		t.renderNode(w, child, newIndent, i == len(kids)-1, false)
	}
}
