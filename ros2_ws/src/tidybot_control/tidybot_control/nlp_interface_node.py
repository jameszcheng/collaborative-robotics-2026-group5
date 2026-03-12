#!/usr/bin/env python3
"""
NLP Interface Node for TidyBot2.

Provides a ROS2 service and interactive terminal for natural-language robot control.
Uses Google Gemini to parse text commands into structured task specifications.

Service:
    /nlp/parse (tidybot_msgs/srv/NlpCommand)
        - text_input: natural language command
        - Returns: type, intent, object, target, message, raw_json

Topics Published:
    /nlp/response (std_msgs/String) - JSON string of each LLM response
    /perception/target_label (std_msgs/String) - target object label for perception

Topics Subscribed:
    /coordinator/status (std_msgs/String) - pick-and-place pipeline state updates
    /perception/object_found (std_msgs/Bool) - detector object visibility
    /perception/object_confidence (std_msgs/Float32) - detector confidence
    /perception/object_label (std_msgs/String) - detector label
    /coordinator/pickup_complete (std_msgs/Bool) - pickup result

Parameters:
    - gemini_api_key (string): Gemini API key (default: from GEMINI_API_KEY env var)
    - use_voice (bool, default true): Enable microphone input via /microphone/record service
    - use_tts (bool, default true): Enable text-to-speech output
    - schema_path (string): Path to task_schema.json (default: auto-detect from package)
    - interactive (bool, default true): Run interactive terminal loop

Usage:
    # As ROS2 node with interactive terminal:
    ros2 run tidybot_control nlp_interface_node

    # Service-only mode (no terminal):
    ros2 run tidybot_control nlp_interface_node --ros-args -p interactive:=false

    # Call the service:
    ros2 service call /nlp/parse tidybot_msgs/srv/NlpCommand "{text_input: 'pick up the apple'}"
"""

import json
import sys
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
try:
    from tidybot_msgs.srv import AudioRecord
    try:
        from tidybot_msgs.srv import NlpCommand  # type: ignore
    except ImportError:
        from tidybot_msgs.srv import NLPCommand as NlpCommand  # type: ignore
except ImportError:
    # Some generated tidybot_msgs Python builds omit the service symbol from
    # srv/__init__.py; also support ROS name generation that uses NLPCommand.
    from tidybot_msgs.srv import AudioRecord
    try:
        from tidybot_msgs.srv._nlp_command import NlpCommand  # type: ignore
    except ImportError:
        from tidybot_msgs.srv._nlp_command import NLPCommand as NlpCommand  # type: ignore

from tidybot_control.nlp_interface import (
    handle_manual_speech_command,
    manual_speech_hint_lines,
    parse_command,
    set_scene_objects,
    ValidationError,
    speak,
    transcribe_audio,
    show_progress,
    update_schema,
)


class NLPInterfaceNode(Node):
    """ROS2 node wrapping the NLP interface for natural-language robot control."""

    def __init__(self):
        super().__init__('nlp_interface')

        # Parameters
        self.declare_parameter('gemini_api_key', '')
        self.declare_parameter('use_voice', True)
        self.declare_parameter('use_tts', True)
        self.declare_parameter('schema_path', '')
        self.declare_parameter('interactive', True)

        self.api_key = self.get_parameter('gemini_api_key').value
        self.use_voice = self.get_parameter('use_voice').value
        self.use_tts = self.get_parameter('use_tts').value
        schema_path = self.get_parameter('schema_path').value
        self.interactive = self.get_parameter('interactive').value

        # Load custom schema if provided
        if schema_path:
            try:
                update_schema(schema_path)
                self.get_logger().info(f'Loaded schema from: {schema_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load schema: {e}')

        # Conversation history (shared between service and interactive mode)
        self.history = []
        self.history_lock = threading.Lock()

        # Service: /nlp/parse
        self.srv = None
        try:
            self.srv = self.create_service(
                NlpCommand, '/nlp/parse', self._parse_callback
            )
        except Exception as e:
            # Continue in interactive/topic-only mode if message generation is broken.
            self.get_logger().error(
                f'Failed to create /nlp/parse service (NlpCommand type support unavailable): {e}'
            )

        # Publisher: /nlp/response
        self.response_pub = self.create_publisher(String, '/nlp/response', 10)
        self.perception_target_pub = self.create_publisher(String, '/perception/target_label', 10)
        self.create_subscription(String, '/coordinator/status', self._coordinator_status_cb, 10)
        self.create_subscription(Bool, '/perception/object_found', self._object_found_cb, 10)
        self.create_subscription(Float32, '/perception/object_confidence', self._object_confidence_cb, 10)
        self.create_subscription(String, '/perception/object_label', self._object_label_cb, 10)
        self.create_subscription(Bool, '/coordinator/pickup_complete', self._pickup_complete_cb, 10)

        self._last_pipeline_status = None
        self._last_pipeline_state = ''
        self._pipeline_label = ''
        self._object_found = False
        self._object_confidence = 0.0
        self._object_label = ''
        self._scene_connected = False
        self._announced_search_detection = False
        self._announced_redetect_detection = False
        self._update_scene_cache()

        # Microphone service client (for voice input)
        self.mic_client = None
        if self.use_voice:
            self.mic_client = self.create_client(AudioRecord, '/microphone/record')
            if self.mic_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('Microphone service connected')
            else:
                self.get_logger().warn(
                    'Microphone service not available — voice input disabled'
                )
                self.mic_client = None

        self.get_logger().info('NLP interface node ready')
        self.get_logger().info(f'  Voice input: {"enabled" if self.mic_client else "disabled"}')
        self.get_logger().info(f'  TTS output: {"enabled" if self.use_tts else "disabled"}')
        self.get_logger().info(f'  Interactive: {"enabled" if self.interactive else "disabled"}')

        if self.interactive and not sys.stdin.isatty():
            self.get_logger().warn(
                'Interactive terminal input is unavailable under this launch context '
                '(stdin is not a TTY). Use /nlp/parse or run `ros2 run tidybot_control '
                'nlp_interface_node` in its own terminal for push-to-talk.'
            )
            self.interactive = False

        # Start interactive terminal in a background thread
        if self.interactive:
            self._terminal_thread = threading.Thread(
                target=self._interactive_loop, daemon=True
            )
            self._terminal_thread.start()

    def _parse_callback(self, request, response):
        """Handle NlpCommand service calls."""
        text = request.text_input.strip()
        if not text:
            response.success = False
            response.type = ''
            response.message = 'Empty input'
            response.raw_json = ''
            return response

        try:
            with self.history_lock:
                result = parse_command(text, self.history, api_key=self.api_key or None)
                raw_json = json.dumps(result)
                self.history.append({"user": text, "assistant": raw_json})

            response.success = True
            response.type = result.get("type", "")
            response.intent = result.get("intent", "")
            response.object = result.get("object", "")
            response.target = result.get("target", "")
            response.message = result.get("message", "")
            response.raw_json = raw_json

            # Publish to topic
            msg = String()
            msg.data = raw_json
            self.response_pub.publish(msg)
            self._publish_perception_target_if_present(result)

            self.get_logger().info(f'Parsed: "{text}" → {result["type"]}')

        except ValidationError as e:
            response.success = False
            response.type = 'error'
            response.message = str(e)
            response.raw_json = ''
            self.get_logger().error(f'Validation error: {e}')

        except Exception as e:
            response.success = False
            response.type = 'error'
            response.message = str(e)
            response.raw_json = ''
            self.get_logger().error(f'Parse error: {e}')

        return response

    def _record_and_transcribe(self) -> str:
        """Use the microphone service to record audio and transcribe it."""
        if self.mic_client is None:
            print("  [Voice input unavailable: microphone service not connected]")
            return ""

        # Start recording
        print("  [Push-to-talk: starting microphone recording]")
        req = AudioRecord.Request()
        req.start = True
        future = self.mic_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn('Failed to start microphone recording')
            return ""

        # Wait for user to press Enter to stop
        print("  [Recording... press Enter to stop]")
        try:
            input()
        except (KeyboardInterrupt, EOFError):
            return ""

        # Stop recording and get audio
        req2 = AudioRecord.Request()
        req2.start = False
        future2 = self.mic_client.call_async(req2)
        rclpy.spin_until_future_complete(self, future2, timeout_sec=10.0)
        result2 = future2.result()
        if result2 is None or not result2.success:
            self.get_logger().warn('Failed to stop microphone recording')
            return ""

        if not result2.audio_data:
            print("  [No audio captured]")
            return ""

        # Transcribe
        print("  [Transcribing...]")
        audio = np.array(result2.audio_data, dtype=np.float32)
        text = transcribe_audio(audio, sample_rate=result2.sample_rate)
        return text

    def _speak(self, text: str):
        """Speak text asynchronously so TTS failures do not affect ROS callbacks."""
        if not self.use_tts or not text:
            return
        threading.Thread(target=self._safe_speak, args=(text,), daemon=True).start()

    def _safe_speak(self, text: str):
        """Run TTS with exception isolation."""
        try:
            speak(text)
        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')

    def _publish_perception_target_if_present(self, result: dict):
        """Publish requested object label so perception can search for it."""
        if result.get("type") != "command":
            return
        obj = str(result.get("object", "")).strip().lower()
        if not obj or obj == "unknown":
            return
        msg = String()
        msg.data = obj
        self.perception_target_pub.publish(msg)
        self.get_logger().info(f'Perception target set: "{obj}"')

    def _coordinator_status_cb(self, msg: String):
        """Relay coordinator state changes back through the NLP interface."""
        status = msg.data.strip()
        if not status or status == self._last_pipeline_status:
            return

        self._last_pipeline_status = status
        state, _, label = status.partition(':')
        state = state.strip()
        label = label.strip()
        prev_state = self._last_pipeline_state
        self._last_pipeline_state = state
        self._pipeline_label = label

        if state == 'SEARCHING':
            self._announced_search_detection = False
        elif state == 'REDETECTING':
            self._announced_redetect_detection = False

        pipeline_text = self._format_pipeline_status(state, label, prev_state)
        self.get_logger().info(f'Pipeline status: {status}')
        if pipeline_text:
            print(f'Pipeline: {pipeline_text}')
            self._speak(pipeline_text)

    @staticmethod
    def _format_pipeline_status(state: str, label: str, prev_state: str) -> str:
        """Convert coordinator state updates into short operator-facing messages."""
        if state == 'IDLE':
            if not prev_state:
                return ''
            if prev_state == 'DONE':
                return 'Pipeline idle. Ready for the next command.'
            return 'Pipeline idle.'
        if state == 'SEARCHING':
            return f"Searching for {label}." if label else 'Searching for target object.'
        if state == 'NAVIGATING':
            return f"Navigating to {label}." if label else 'Navigating to target.'
        if state == 'REDETECTING':
            return f"Refining the pose for {label}." if label else 'Refining the object pose.'
        if state == 'PICKING_UP':
            return f"Picking up {label}." if label else 'Picking up the object.'
        if state == 'DONE':
            return f"Finished handling {label}." if label else 'Task complete.'
        if state == 'FAILED':
            return f"Pipeline failed for {label}." if label else 'Pipeline failed.'
        return f'Pipeline state changed to {state.lower()}.' if state else ''

    def _object_found_cb(self, msg: Bool):
        self._scene_connected = True
        self._object_found = msg.data
        self._update_scene_cache()
        self._maybe_announce_detection()

    def _object_confidence_cb(self, msg: Float32):
        self._scene_connected = True
        self._object_confidence = float(msg.data)
        self._update_scene_cache()
        self._maybe_announce_detection()

    def _object_label_cb(self, msg: String):
        self._scene_connected = True
        self._object_label = msg.data.strip().lower()
        self._update_scene_cache()
        self._maybe_announce_detection()

    def _pickup_complete_cb(self, msg: Bool):
        if not msg.data:
            text = (
                f"Pickup failed for {self._pipeline_label}."
                if self._pipeline_label else 'Pickup failed.'
            )
            print(f'Pipeline: {text}')
            self._speak(text)

    def _maybe_announce_detection(self):
        """Announce object acquisition once per relevant coordinator phase."""
        if not self._object_found or self._object_confidence < 0.4:
            return

        active_label = self._pipeline_label or self._object_label
        if self._last_pipeline_state == 'SEARCHING' and not self._announced_search_detection:
            self._announced_search_detection = True
            text = (
                f"I found {active_label}."
                if active_label else 'I found the target object.'
            )
            print(f'Pipeline: {text}')
            self._speak(text)
        elif self._last_pipeline_state == 'REDETECTING' and not self._announced_redetect_detection:
            self._announced_redetect_detection = True
            text = (
                f"I see {active_label} again. Refining the grasp pose."
                if active_label else 'I see the object again. Refining the grasp pose.'
            )
            print(f'Pipeline: {text}')
            self._speak(text)

    def _update_scene_cache(self):
        """Expose the latest detector state to the prompt builder used by parse_command."""
        if not self._scene_connected:
            set_scene_objects([], connected=False)
            return
        if self._object_found and self._object_label and self._object_confidence > 0.0:
            set_scene_objects([{
                'object': self._object_label,
                'confidence': self._object_confidence,
            }], connected=True)
            return
        set_scene_objects([], connected=True)

    def _process_input(self, command: str) -> bool:
        """Process a single user input. Returns False if should exit."""
        if handle_manual_speech_command(command):
            return True

        try:
            with self.history_lock:
                result = parse_command(command, self.history, api_key=self.api_key or None)
                raw_json = json.dumps(result)

            # Publish to topic
            msg = String()
            msg.data = raw_json
            self.response_pub.publish(msg)
            self._publish_perception_target_if_present(result)

            if result["type"] == "exit":
                print(f"Robot: {result['message']}")
                self._speak(result["message"])
                with self.history_lock:
                    self.history.append({"user": command, "assistant": raw_json})
                return False

            elif result["type"] == "chat":
                print(f"Robot: {result['message']}")
                self._speak(result["message"])

            elif result["type"] == "confirm":
                print(f"Robot: {result['message']}")
                self._speak(result["message"])
                with self.history_lock:
                    self.history.append({"user": command, "assistant": raw_json})

                # Get confirmation
                confirm = self._get_input()
                if confirm is None:
                    return False
                if not confirm:
                    return True

                # Send confirmation to LLM
                with self.history_lock:
                    result2 = parse_command(confirm, self.history, api_key=self.api_key or None)
                    raw_json2 = json.dumps(result2)
                    self.history.append({"user": confirm, "assistant": raw_json2})

                # Publish confirmation response
                msg2 = String()
                msg2.data = raw_json2
                self.response_pub.publish(msg2)
                self._publish_perception_target_if_present(result2)

                if result2["type"] == "command":
                    task_spec = {k: v for k, v in result2.items() if k != "type"}
                    print("Robot: Command received.")
                    print(json.dumps(task_spec, indent=2))
                    print("Robot: Use keys 1-7 if you want me to narrate the steps manually.")
                elif result2["type"] == "chat":
                    print(f"Robot: {result2['message']}")
                    self._speak(result2["message"])

                return True

            elif result["type"] == "command":
                task_spec = {k: v for k, v in result.items() if k != "type"}
                print("Robot: Command received.")
                print(json.dumps(task_spec, indent=2))
                print("Robot: Use keys 1-7 if you want me to narrate the steps manually.")

            with self.history_lock:
                self.history.append({"user": command, "assistant": raw_json})

        except ValidationError as e:
            self.get_logger().error(f'Validation error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        return True

    def _get_input(self) -> str:
        """Get user input via text or voice. Returns None on exit signal."""
        try:
            raw = input("\nYou (type or press Enter to talk): ")
            command = raw.strip()
        except (KeyboardInterrupt, EOFError):
            return None

        if not command and self.mic_client:
            print("  [Push-to-talk triggered: speak after recording starts]")
            command = self._record_and_transcribe()
            if not command:
                print("  [No speech detected, try again]")
                return ""
            print(f'  [Heard: "{command}"]')

        return command

    def _safe_show_progress(self, task_spec: dict):
        """Keep UI/progress helper failures local to the NLP node."""
        try:
            show_progress(task_spec)
        except Exception as e:
            self.get_logger().error(f'Progress display error: {e}')

    def _interactive_loop(self):
        """Interactive terminal loop running in a background thread."""
        print("\nRobot Assistant (Ctrl+C to exit)")
        print("=" * 60)
        if self.mic_client:
            print("  [Enter]  = Push-to-talk (press Enter to start, Enter to stop)")
        print("  [type]   = Type a message and press Enter")
        for line in manual_speech_hint_lines():
            print(line)
        print("  [Service] /nlp/parse also available for programmatic access")
        print("=" * 60)

        while rclpy.ok():
            command = self._get_input()
            if command is None:
                self._speak("Goodbye!")
                print("\nGoodbye!")
                break
            if not command:
                continue
            if not self._process_input(command):
                break


def main(args=None):
    rclpy.init(args=args)
    node = NLPInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
