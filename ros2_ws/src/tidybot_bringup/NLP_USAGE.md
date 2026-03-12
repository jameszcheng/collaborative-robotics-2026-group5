# NLP Interface Usage

This note explains how to use the NLP interface during the banana-to-bin demo.
The goal is to make the robot sound conversational and responsive while still
driving the same underlying pick-and-place behavior.

## What the NLP Interface Does

The NLP interface has two jobs:

1. Turn speech or typed text into a structured robot command.
2. Speak back to the user with natural progress updates.

In the full ROS setup, the NLP node connects to perception and coordinator topics.
That means it can:

- accept commands like "pick up the banana"
- confirm the task before execution
- trigger the robot pipeline
- speak progress as the robot searches, navigates, grasps, and finishes

In the standalone terminal mode, it still works as a conversational front end, and
you can manually trigger spoken status lines from the keyboard.

## Two Ways to Run It

### 1. ROS-connected mode

Use this when the robot pipeline is running:

```bash
cd ros2_ws
source setup_env.bash
export GEMINI_API_KEY="YOUR_KEY"
ros2 run tidybot_control nlp_interface_node
```

This mode publishes and subscribes to ROS topics, so speech can follow the real robot state.

### 2. Standalone terminal mode

Use this when you only want the conversational interface:

```bash
cd ros2_ws/src/tidybot_control/tidybot_control
export GEMINI_API_KEY="YOUR_KEY"
python nlp_interface.py
```

This mode does not depend on ROS topics. It is useful for rehearsing the conversation
or keeping the interaction going if the physical pipeline is not active.

## Manual Speech Keys

In the interactive terminal, you can type these keys at any time to make the robot speak:

- `1` = `I found the banana. Target locked.`
- `2` = `I'm gliding into position now.`
- `3` = `I have a clean view. Lining up the grasp.`
- `4` = `Going for the pickup now.`
- `5` = `Banana secured. Heading to the bin.`
- `6` = `Placing the banana in the bin now.`
- `7` = `Task complete. The banana is in the bin.`
- `8` = `That attempt didn't land cleanly, but I'm ready to try again.`

These are useful as a live fallback if the robot pauses, misses a detection, or you
want tighter control over what the audience hears.

## Short Conversation Script

This is a simple, natural flow that works well for the demo.

Human:
`Hi, how are you?`

Robot:
`Hi. Good to see you. What would you like me to do?`

Human:
`What do you see?`

Robot:
`I can see a banana sitting here.`

Human:
`Can you clean it up?`

Robot:
`Absolutely. I can take care of that and move it to the bin. Want me to start?`

Human:
`Yes.`

Robot:
`Command received.`

```json
{
  "intent": "pick_and_place",
  "object": "banana",
  "target": "bin"
}
```

Robot:
`Use keys 1-7 if you want me to narrate the steps manually.`

Then the robot can continue with real or manual progress lines:

- `I found the banana. Target locked.`
- `I'm gliding into position now.`
- `I have a clean view. Lining up the grasp.`
- `Going for the pickup now.`
- `Banana secured. Heading to the bin.`
- `Placing the banana in the bin now.`
- `Task complete. The banana is in the bin.`
