#!/usr/bin/env python3
"""
NLP Interface Node for TidyBot2.

Provides a ROS2 service and interactive terminal for natural-language robot control.
Uses Google Gemini to parse text commands into structured task specifications.

Service:
    /nlp/parse (tidybot_msgs/srv/NLPCommand)
        - text_input: natural language command
        - Returns: type, intent, object, target, message, raw_json

Topics Published:
    /nlp/response (std_msgs/String) - JSON string of each LLM response
    /perception/target_label (std_msgs/String) - target object label for perception

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
    ros2 service call /nlp/parse tidybot_msgs/srv/NLPCommand "{text_input: 'pick up the apple'}"
"""

import json
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tidybot_msgs.srv import NLPCommand, AudioRecord

from tidybot_control.nlp_interface import (
    parse_command,
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
        self.srv = self.create_service(
            NLPCommand, '/nlp/parse', self._parse_callback
        )

        # Publisher: /nlp/response
        self.response_pub = self.create_publisher(String, '/nlp/response', 10)
        self.perception_target_pub = self.create_publisher(String, '/perception/target_label', 10)

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

        # Start interactive terminal in a background thread
        if self.interactive:
            self._terminal_thread = threading.Thread(
                target=self._interactive_loop, daemon=True
            )
            self._terminal_thread.start()

    def _parse_callback(self, request, response):
        """Handle NLPCommand service calls."""
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
            return ""

        # Start recording
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
            return ""

        # Transcribe
        print("  [Transcribing...]")
        audio = np.array(result2.audio_data, dtype=np.float32)
        text = transcribe_audio(audio, sample_rate=result2.sample_rate)
        return text

    def _speak(self, text: str):
        """Speak text if TTS is enabled."""
        if self.use_tts:
            speak(text)

    def _publish_perception_target_if_present(self, result: dict):
        """Publish requested object label so perception can search for it."""
        if result.get("type") not in ("confirm", "command"):
            return
        obj = str(result.get("object", "")).strip().lower()
        if not obj or obj == "unknown":
            return
        msg = String()
        msg.data = obj
        self.perception_target_pub.publish(msg)
        self.get_logger().info(f'Perception target set: "{obj}"')

    def _process_input(self, command: str) -> bool:
        """Process a single user input. Returns False if should exit."""
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
                    print(f"Robot: Executing task!")
                    print(json.dumps(task_spec, indent=2))
                    show_progress(task_spec)
                elif result2["type"] == "chat":
                    print(f"Robot: {result2['message']}")
                    self._speak(result2["message"])

                return True

            elif result["type"] == "command":
                task_spec = {k: v for k, v in result.items() if k != "type"}
                print(f"Robot: Executing task!")
                print(json.dumps(task_spec, indent=2))
                show_progress(task_spec)

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
            command = self._record_and_transcribe()
            if not command:
                print("  [No speech detected, try again]")
                return ""
            print(f'  [Heard: "{command}"]')

        return command

    def _interactive_loop(self):
        """Interactive terminal loop running in a background thread."""
        print("\nRobot Assistant (Ctrl+C to exit)")
        print("=" * 60)
        if self.mic_client:
            print("  [Enter]  = Push-to-talk (press Enter to start, Enter to stop)")
        print("  [type]   = Type a message and press Enter")
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
