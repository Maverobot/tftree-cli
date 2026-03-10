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

	tfSub, err := tf2_msgs_msg.NewTFMessageSubscription(
		node, "/tf", nil, makeCallback(),
	)
	if err != nil {
		return fmt.Errorf("could not subscribe to /tf: %w", err)
	}
	defer tfSub.Close()

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
