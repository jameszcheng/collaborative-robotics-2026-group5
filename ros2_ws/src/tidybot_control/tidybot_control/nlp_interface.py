"""
Language-understanding module for robotics system.
Converts natural-language text commands into structured JSON task specifications.

Supported intent: pick_and_place
Supported object: apple
Supported targets: bin, table
"""

import io
import json
import os
import re
import subprocess
import sys
import tempfile
import threading
import time

# Load .env file if present (for GEMINI_API_KEY, etc.)
def _load_dotenv():
    """Load key=value pairs from .env file into os.environ."""
    # Search: repo root, cwd, home
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '.env'),
        os.path.join(os.getcwd(), '.env'),
        os.path.expanduser('~/.env'),
    ]
    for path in candidates:
        path = os.path.realpath(path)
        if os.path.isfile(path):
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    key, _, value = line.partition('=')
                    key, value = key.strip(), value.strip().strip('"').strip("'")
                    if key and key not in os.environ:
                        os.environ[key] = value
            break

_load_dotenv()

import numpy as np

try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except (ImportError, OSError):
    sd = None
    HAS_SOUNDDEVICE = False

try:
    import soundfile as sf
    HAS_SOUNDFILE = True
except (ImportError, OSError):
    sf = None
    HAS_SOUNDFILE = False

from google import genai
from google.genai import types


# --- Schema Loading ---

def load_schema(schema_path: str) -> dict:
    """Load task schema from a JSON file."""
    with open(schema_path, "r") as f:
        return json.load(f)


def get_default_schema_path() -> str:
    """Return the default schema path for both source and installed ROS layouts."""
    # Installed ROS2 packages place config files under share/<pkg>/config, not site-packages.
    try:
        from ament_index_python.packages import get_package_share_directory

        share_schema = os.path.join(
            get_package_share_directory("tidybot_control"),
            "config",
            "task_schema.json",
        )
        if os.path.exists(share_schema):
            return share_schema
    except Exception:
        pass

    # Source-tree fallback (useful for standalone/local execution).
    return os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "config",
        "task_schema.json",
    )


# --- Schema Constants (can be overridden via load_schema) ---

_default_schema_path = get_default_schema_path()
if os.path.exists(_default_schema_path):
    _schema = load_schema(_default_schema_path)
else:
    # Fallback for standalone usage
    _schema = {"intent": ["pick_and_place"], "object": ["apple", "unknown"], "target": ["bin", "bowl", "table", "unknown"]}

VALID_INTENTS = set(_schema["intent"])
VALID_OBJECTS = set(_schema["object"])
VALID_TARGETS = set(_schema["target"])


def update_schema(schema_path: str):
    """Update the global schema constants from a new schema file."""
    global VALID_INTENTS, VALID_OBJECTS, VALID_TARGETS
    schema = load_schema(schema_path)
    VALID_INTENTS = set(schema["intent"])
    VALID_OBJECTS = set(schema["object"])
    VALID_TARGETS = set(schema["target"])


# --- Perception Interface ---

def get_scene_objects():
    """Return detected objects from the perception/vision system.

    Replace this stub with your actual perception pipeline (e.g. ROS2 topic,
    camera detection, etc.). Return a list of dicts with 'object' and 'location'
    keys, or None if perception is not available.
    """
    return None  # No perception connected yet


def _build_perception_context() -> str:
    """Build a context string describing what the robot can currently see."""
    objects = get_scene_objects()
    if objects is None:
        return (
            "\n[Perception status: NOT CONNECTED. You do not know what objects are "
            "on the table. If the user asks you to interact with objects, ask them "
            "to tell you what object and where it is.]\n"
        )
    if not objects:
        return "\n[Perception status: CONNECTED. The scene is empty — no objects detected.]\n"
    desc = ", ".join(f'{o["object"]} (on {o["location"]})' for o in objects)
    return f"\n[Perception status: CONNECTED. Objects currently visible: {desc}]\n"


# --- System Prompt (built dynamically from schema) ---

def build_system_prompt() -> str:
    """Build the system prompt from current schema constants."""
    _intents_str = " | ".join(f'"{v}"' for v in sorted(VALID_INTENTS))
    _objects_str = " | ".join(f'"{v}"' for v in sorted(VALID_OBJECTS - {"unknown"}))
    _targets_str = " | ".join(f'"{v}"' for v in sorted(VALID_TARGETS - {"unknown"}))

    return (
        'You are a friendly robot assistant for a pick-and-place system. Your name is "Robo" by default, '
        "but if the user gives you a name or nickname, you MUST remember it and use it. "
        "You MUST pay close attention to the full conversation history. If the user told you something "
        "earlier (their name, your name, preferences, etc.), you MUST remember and reference it accurately.\n"
        "You can have natural conversations with the user AND parse robot commands.\n\n"

        "You respond with ONLY a raw JSON object (no code fences, no extra text). There are 4 response types:\n\n"

        '1) CHAT — for casual conversation:\n'
        '{"type": "chat", "message": "<your friendly response>"}\n\n'

        '2) CONFIRM — when the user gives a robot command, ALWAYS ask for confirmation first:\n'
        '{"type": "confirm", "intent": "<intent>", "object": "<object>", "target": "<target>", '
        '"message": "<friendly confirmation, e.g. I will pick up the apple and place it in the bin. Should I go ahead?>"}\n\n'

        '3) COMMAND — ONLY after the user says yes/sure/go ahead to a confirm:\n'
        '{"type": "command", "intent": "<intent>", "object": "<object>", "target": "<target>"}\n\n'

        '4) EXIT — when the user says goodbye, bye, see you later, I\'m done, etc.:\n'
        '{"type": "exit", "message": "<friendly goodbye message>"}\n\n'

        "Allowed values:\n"
        f"- intent: {_intents_str}\n"
        f"- object: {_objects_str} | \"unknown\"\n"
        f"- target: {_targets_str} | \"unknown\"\n\n"

        "Rules:\n"
        "- NEVER return a command directly. ALWAYS return a confirm first, then command only after user says yes.\n"
        "- If the user says no/cancel to a confirm, respond with chat acknowledging the cancellation.\n"
        "- If the object or target is missing/unclear, respond with chat to ask for clarification.\n"
        "- For vague requests like 'clean up the table' or 'tidy up', break it down: "
        "figure out what objects could be involved and ask the user step by step. "
        'For example: {"type": "chat", "message": "I can help clean up! I see an apple on the table. Should I move it to the bin?"}\n'
        "- Only return confirm/command when ALL fields (object and target) are clear.\n"
        "- Match the user's words to the closest allowed object and target values.\n\n"

        "Rules for chat:\n"
        "- Be friendly and conversational.\n"
        "- Remember all details from conversation history (user's name, your nickname, preferences, etc.).\n"
        "- If the user gives you a name, adopt it. If they tell you their name, use it.\n\n"

        "Rules for exit:\n"
        "- When the user says goodbye, bye, see you, I'm done, that's all, etc., respond with an exit type.\n"
        "- Include a warm, friendly goodbye message.\n\n"

        "Output ONLY the raw JSON object. No code fences, no commentary, no extra text.\n"
    )


# --- Validation ---

class ValidationError(Exception):
    """Raised when LLM output fails schema validation."""
    pass


def validate_response(raw_output: str) -> dict:
    """Parse and validate LLM output as either a chat message or a task command.

    Args:
        raw_output: Raw string output from the LLM.

    Returns:
        Validated response dict with "type" key ("chat" or "command").

    Raises:
        ValidationError: If the output is invalid JSON or violates the schema.
    """
    cleaned = raw_output.strip()
    if cleaned.startswith("```"):
        lines = cleaned.split("\n")
        lines = [l for l in lines if not l.strip().startswith("```")]
        cleaned = "\n".join(lines).strip()

    try:
        spec = json.loads(cleaned)
    except json.JSONDecodeError as e:
        raise ValidationError(f"Invalid JSON from LLM: {e}\nRaw output: {raw_output!r}")

    if not isinstance(spec, dict):
        raise ValidationError(f"Expected JSON object, got {type(spec).__name__}")

    if "type" not in spec:
        raise ValidationError("Missing 'type' key in response")

    if spec["type"] == "chat":
        if "message" not in spec:
            raise ValidationError("Chat response missing 'message' key")
        return spec

    if spec["type"] == "exit":
        if "message" not in spec:
            raise ValidationError("Exit response missing 'message' key")
        return spec

    if spec["type"] == "confirm":
        required = {"intent", "object", "target", "message"}
        missing = required - set(spec.keys())
        if missing:
            raise ValidationError(f"Confirm response missing keys: {missing}")
        if spec["intent"] not in VALID_INTENTS:
            raise ValidationError(
                f"Invalid intent: {spec['intent']!r}. Allowed: {VALID_INTENTS}"
            )
        if spec["object"] not in VALID_OBJECTS:
            raise ValidationError(
                f"Invalid object: {spec['object']!r}. Allowed: {VALID_OBJECTS}"
            )
        if spec["target"] not in VALID_TARGETS:
            raise ValidationError(
                f"Invalid target: {spec['target']!r}. Allowed: {VALID_TARGETS}"
            )
        return spec

    if spec["type"] == "command":
        required = {"intent", "object", "target"}
        missing = required - set(spec.keys())
        if missing:
            raise ValidationError(f"Missing required keys: {missing}")

        if spec["intent"] not in VALID_INTENTS:
            raise ValidationError(
                f"Invalid intent: {spec['intent']!r}. Allowed: {VALID_INTENTS}"
            )
        if spec["object"] not in VALID_OBJECTS:
            raise ValidationError(
                f"Invalid object: {spec['object']!r}. Allowed: {VALID_OBJECTS}"
            )
        if spec["target"] not in VALID_TARGETS:
            raise ValidationError(
                f"Invalid target: {spec['target']!r}. Allowed: {VALID_TARGETS}"
            )
        return spec

    raise ValidationError(f"Unknown type: {spec['type']!r}. Must be 'chat', 'confirm', 'command', or 'exit'.")


# --- LLM Interface ---

def _find_allowed_value(text: str, allowed: set) -> str:
    """Find the first allowed value present in text as a whole token/phrase."""
    candidates = sorted((v for v in allowed if v != "unknown"), key=len, reverse=True)
    for value in candidates:
        pattern = r"\b" + re.escape(value.lower()) + r"\b"
        if re.search(pattern, text):
            return value
    return "unknown"


def _looks_like_pick_and_place(text: str) -> bool:
    verbs = ("pick", "grab", "take", "place", "put", "move")
    return any(v in text for v in verbs)


def _extract_slots_from_text(command: str) -> dict:
    """Extract intent/object/target from direct language cues."""
    text = command.strip().lower()
    return {
        "intent": "pick_and_place" if _looks_like_pick_and_place(text) else "",
        "object": _find_allowed_value(text, VALID_OBJECTS),
        "target": _find_allowed_value(text, VALID_TARGETS),
    }


def _is_affirmative(command: str) -> bool:
    text = command.strip().lower()
    return text in {"yes", "y", "yeah", "yep", "sure", "ok", "okay", "go ahead", "do it"}


def _last_confirm_from_history(history: list) -> dict:
    for turn in reversed(history):
        raw = turn.get("assistant", "")
        if not raw:
            continue
        try:
            data = json.loads(raw)
        except Exception:
            continue
        if data.get("type") == "confirm":
            return data
    return {}


def _apply_deterministic_fallback(command: str, history: list, parsed: dict) -> dict:
    slots = _extract_slots_from_text(command)
    has_full_slots = (
        slots["intent"] in VALID_INTENTS
        and slots["object"] in VALID_OBJECTS
        and slots["object"] != "unknown"
        and slots["target"] in VALID_TARGETS
        and slots["target"] != "unknown"
    )

    # If user gave a clear task but LLM drifted to chat/mismatch, force a schema-valid confirm.
    if has_full_slots and parsed.get("type") not in {"confirm", "command"}:
        return {
            "type": "confirm",
            "intent": slots["intent"],
            "object": slots["object"],
            "target": slots["target"],
            "message": f'I will pick up the {slots["object"]} and place it in the {slots["target"]}. Should I go ahead?',
        }

    # Keep extracted slots authoritative when explicitly present in the user's utterance.
    if parsed.get("type") in {"confirm", "command"} and has_full_slots:
        parsed["intent"] = slots["intent"]
        parsed["object"] = slots["object"]
        parsed["target"] = slots["target"]
        if parsed["type"] == "confirm":
            parsed["message"] = (
                f'I will pick up the {slots["object"]} and place it in the {slots["target"]}. '
                "Should I go ahead?"
            )
        return parsed

    # If user confirms with "yes", turn latest confirm into a command even if LLM responds oddly.
    if _is_affirmative(command):
        last_confirm = _last_confirm_from_history(history)
        if last_confirm:
            return {
                "type": "command",
                "intent": last_confirm.get("intent", "pick_and_place"),
                "object": last_confirm.get("object", "unknown"),
                "target": last_confirm.get("target", "unknown"),
            }

    return parsed


def parse_command(command: str, history: list, api_key: str = None) -> dict:
    """Send user input to the LLM with conversation history and return a validated response."""
    if api_key is None:
        api_key = os.environ.get("GEMINI_API_KEY", "")
    client = genai.Client(api_key=api_key)

    contents = []
    for turn in history:
        contents.append(types.Content(role="user", parts=[types.Part(text=turn["user"])]))
        contents.append(types.Content(role="model", parts=[types.Part(text=turn["assistant"])]))
    contents.append(types.Content(role="user", parts=[types.Part(text=command)]))

    # Inject perception context into the system prompt dynamically
    perception_context = _build_perception_context()
    # Build from current schema so runtime schema overrides are reflected in the prompt.
    full_prompt = build_system_prompt() + perception_context

    response = client.models.generate_content(
        model="gemini-2.5-flash",
        contents=contents,
        config=types.GenerateContentConfig(
            system_instruction=full_prompt,
            temperature=0,
            max_output_tokens=200,
        ),
    )

    raw_output = response.text
    parsed = validate_response(raw_output)
    return _apply_deterministic_fallback(command, history, parsed)


# --- Text-to-Speech ---

def speak(text: str):
    """Speak text out loud. Works on macOS, Linux, and Windows."""
    import platform
    system = platform.system()
    try:
        if system == "Darwin":
            subprocess.run(["say", text])
        elif system == "Linux":
            subprocess.run(["espeak", text])
        elif system == "Windows":
            ps_cmd = f'Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak("{text}")'
            subprocess.run(["powershell", "-Command", ps_cmd])
    except FileNotFoundError:
        pass  # TTS not available on this system, skip silently


# --- Speech-to-Text ---

SAMPLE_RATE = 16000


def transcribe_audio(audio_data: np.ndarray, sample_rate: int = SAMPLE_RATE) -> str:
    """Transcribe audio data using the speech_recognition library.

    Args:
        audio_data: Numpy array of audio samples (float32 or int16).
        sample_rate: Audio sample rate in Hz.

    Returns:
        Transcribed text, or empty string on failure.
    """
    if len(audio_data) == 0:
        return ""

    if not HAS_SOUNDFILE:
        return ""

    # Convert to int16 WAV for speech_recognition
    if audio_data.dtype == np.float32:
        audio_int16 = (audio_data * 32767).astype(np.int16)
    elif audio_data.dtype == np.int16:
        audio_int16 = audio_data
    else:
        audio_int16 = audio_data.astype(np.int16)

    # Save to a temporary WAV file
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    sf.write(tmp.name, audio_int16, sample_rate)
    tmp.close()

    try:
        import speech_recognition as sr
        recognizer = sr.Recognizer()
        with sr.AudioFile(tmp.name) as source:
            audio = recognizer.record(source)
        text = recognizer.recognize_google(audio)
        return text
    except ImportError:
        return ""
    except Exception:
        return ""
    finally:
        os.unlink(tmp.name)


# --- CLI Entry Point (standalone mode) ---

def record_audio_cli() -> np.ndarray:
    """Record audio from the microphone until the user presses Enter (CLI mode)."""
    if not HAS_SOUNDDEVICE:
        return np.array([], dtype="int16")

    print("  [Recording... press Enter to stop]")
    audio_chunks = []
    stop_event = threading.Event()

    def callback(indata, frames, time_info, status):
        if not stop_event.is_set():
            audio_chunks.append(indata.copy())
    sd.default.device = (0, None)
    stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="int16", callback=callback)
    stream.start()

    input()  # Block until Enter is pressed
    stop_event.set()
    stream.stop()
    stream.close()

    if not audio_chunks:
        return np.array([], dtype="int16")
    return np.concatenate(audio_chunks, axis=0)


def listen_cli() -> str:
    """Push-to-talk: record audio and transcribe it (CLI mode)."""
    audio = record_audio_cli()
    if len(audio) == 0:
        return ""
    print("  [Transcribing...]")
    return transcribe_audio(audio)


def show_progress(task_spec: dict):
    """Simulate task execution with step-by-step progress updates."""
    obj = task_spec.get("object", "object")
    target = task_spec.get("target", "target")

    steps = [
        f"Locating {obj}...",
        f"Moving arm to {obj}...",
        f"Gripping {obj}...",
        f"Lifting {obj}...",
        f"Moving to {target}...",
        f"Placing {obj} on {target}...",
        "Done!",
    ]

    for step in steps:
        print(f"  >> {step}")
        time.sleep(0.8)

    speak(f"Task complete. {obj} has been placed on the {target}.")


def main():
    """Interactive terminal loop with voice and text input (standalone mode)."""
    print("Robot Assistant (Ctrl+C to exit)")
    print("=" * 60)
    print("  [Enter]  = Push-to-talk (press Enter to start, Enter to stop)")
    print("  [type]   = Type a message and press Enter")
    print("=" * 60)

    history = []

    while True:
        try:
            raw = input("\nYou (type or press Enter to talk): ")
            command = raw.strip()

            if not command:
                command = listen_cli()
                if not command:
                    print("  [No speech detected, try again]")
                    continue
                print(f'  [Heard: "{command}"]')
        except (KeyboardInterrupt, EOFError):
            speak("Goodbye!")
            print("\nGoodbye!")
            break

        if not command:
            continue

        try:
            result = parse_command(command, history)
            raw_json = json.dumps(result)

            if result["type"] == "exit":
                print(f"Robot: {result['message']}")
                speak(result["message"])
                break

            elif result["type"] == "chat":
                print(f"Robot: {result['message']}")
                speak(result["message"])

            elif result["type"] == "confirm":
                print(f"Robot: {result['message']}")
                speak(result["message"])
                history.append({"user": command, "assistant": raw_json})

                try:
                    raw2 = input("\nYou (type or press Enter to talk): ")
                    confirm = raw2.strip()
                    if not confirm:
                        confirm = listen_cli()
                        if not confirm:
                            continue
                        print(f'  [Heard: "{confirm}"]')
                except (KeyboardInterrupt, EOFError):
                    speak("Goodbye!")
                    print("\nGoodbye!")
                    break

                if not confirm:
                    continue

                result2 = parse_command(confirm, history)
                raw_json2 = json.dumps(result2)

                if result2["type"] == "command":
                    task_spec = {k: v for k, v in result2.items() if k != "type"}
                    print(f"Robot: Executing task!")
                    print(json.dumps(task_spec, indent=2))
                    show_progress(task_spec)
                elif result2["type"] == "chat":
                    print(f"Robot: {result2['message']}")
                    speak(result2["message"])

                history.append({"user": confirm, "assistant": raw_json2})
                continue

            elif result["type"] == "command":
                task_spec = {k: v for k, v in result.items() if k != "type"}
                print(f"Robot: Executing task!")
                print(json.dumps(task_spec, indent=2))
                show_progress(task_spec)

            history.append({"user": command, "assistant": raw_json})

        except ValidationError as e:
            print(f"[VALIDATION ERROR] {e}", file=sys.stderr)
        except Exception as e:
            print(f"[ERROR] {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
