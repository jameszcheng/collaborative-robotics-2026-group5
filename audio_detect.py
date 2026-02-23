#!/usr/bin/env python3
"""
Standalone NLP + Voice interface for TidyBot2.
No ROS2 dependency — run directly with: python audio_detect.py

Requires:
    pip install google-genai sounddevice soundfile SpeechRecognition numpy

Environment:
    export GEMINI_API_KEY=<your-key>
"""

import json
import os
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except ImportError:
    HAS_SOUNDDEVICE = False

try:
    import soundfile as sf
    HAS_SOUNDFILE = True
except ImportError:
    HAS_SOUNDFILE = False

from google import genai
from google.genai import types


# ── Schema ───────────────────────────────────────────────────────────────────

SCHEMA = {
    "intent": ["pick_and_place"],
    "object": ["apple", "unknown"],
    "target": ["bin", "table", "unknown"],
}

VALID_INTENTS = set(SCHEMA["intent"])
VALID_OBJECTS = set(SCHEMA["object"])
VALID_TARGETS = set(SCHEMA["target"])


# ── System Prompt ────────────────────────────────────────────────────────────

def build_system_prompt() -> str:
    intents_str = " | ".join(f'"{v}"' for v in sorted(VALID_INTENTS))
    objects_str = " | ".join(f'"{v}"' for v in sorted(VALID_OBJECTS - {"unknown"}))
    targets_str = " | ".join(f'"{v}"' for v in sorted(VALID_TARGETS - {"unknown"}))

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
        f"- intent: {intents_str}\n"
        f"- object: {objects_str} | \"unknown\"\n"
        f"- target: {targets_str} | \"unknown\"\n\n"

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

        "\n[Perception status: NOT CONNECTED. You do not know what objects are "
        "on the table. If the user asks you to interact with objects, ask them "
        "to tell you what object and where it is.]\n"
    )


SYSTEM_PROMPT = build_system_prompt()


# ── Validation ───────────────────────────────────────────────────────────────

class ValidationError(Exception):
    pass


def validate_response(raw_output: str) -> dict:
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
        return spec

    if spec["type"] == "command":
        required = {"intent", "object", "target"}
        missing = required - set(spec.keys())
        if missing:
            raise ValidationError(f"Missing required keys: {missing}")
        if spec["intent"] not in VALID_INTENTS:
            raise ValidationError(f"Invalid intent: {spec['intent']!r}. Allowed: {VALID_INTENTS}")
        if spec["object"] not in VALID_OBJECTS:
            raise ValidationError(f"Invalid object: {spec['object']!r}. Allowed: {VALID_OBJECTS}")
        if spec["target"] not in VALID_TARGETS:
            raise ValidationError(f"Invalid target: {spec['target']!r}. Allowed: {VALID_TARGETS}")
        return spec

    raise ValidationError(f"Unknown type: {spec['type']!r}. Must be 'chat', 'confirm', 'command', or 'exit'.")


# ── LLM ──────────────────────────────────────────────────────────────────────

def parse_command(command: str, history: list, api_key: str = None) -> dict:
    if api_key is None:
        api_key = os.environ.get("GEMINI_API_KEY", "")
    client = genai.Client(api_key=api_key)

    contents = []
    for turn in history:
        contents.append(types.Content(role="user", parts=[types.Part(text=turn["user"])]))
        contents.append(types.Content(role="model", parts=[types.Part(text=turn["assistant"])]))
    contents.append(types.Content(role="user", parts=[types.Part(text=command)]))

    response = client.models.generate_content(
        model="gemini-2.5-flash",
        contents=contents,
        config=types.GenerateContentConfig(
            system_instruction=SYSTEM_PROMPT,
            temperature=0,
            max_output_tokens=200,
        ),
    )
    return validate_response(response.text)


# ── Text-to-Speech ───────────────────────────────────────────────────────────

def speak(text: str):
    import platform
    system = platform.system()
    try:
        if system == "Darwin":
            subprocess.run(["say", text])
        elif system == "Linux":
            subprocess.run(["espeak", text])
        elif system == "Windows":
            ps_cmd = (
                f'Add-Type -AssemblyName System.Speech; '
                f'(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak("{text}")'
            )
            subprocess.run(["powershell", "-Command", ps_cmd])
    except FileNotFoundError:
        pass


# ── Speech-to-Text ───────────────────────────────────────────────────────────

SAMPLE_RATE = 16000


def transcribe_audio(audio_data: np.ndarray, sample_rate: int = SAMPLE_RATE) -> str:
    if len(audio_data) == 0 or not HAS_SOUNDFILE:
        return ""

    if audio_data.dtype == np.float32:
        audio_int16 = (audio_data * 32767).astype(np.int16)
    elif audio_data.dtype == np.int16:
        audio_int16 = audio_data
    else:
        audio_int16 = audio_data.astype(np.int16)

    # Flatten to 1-D if needed (sounddevice gives shape (N, 1))
    if audio_int16.ndim > 1:
        audio_int16 = audio_int16.flatten()

    duration = len(audio_int16) / sample_rate
    print(f"  [Audio: {len(audio_int16)} samples, {duration:.1f}s, "
          f"max amplitude: {np.max(np.abs(audio_int16))}]")

    if np.max(np.abs(audio_int16)) < 100:
        print("  [WARNING: Audio is nearly silent — check microphone permissions in System Settings > Privacy & Security > Microphone]")

    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    sf.write(tmp.name, audio_int16, sample_rate)
    tmp.close()

    try:
        import speech_recognition as sr
        recognizer = sr.Recognizer()
        with sr.AudioFile(tmp.name) as source:
            audio = recognizer.record(source)
        return recognizer.recognize_google(audio)
    except ImportError:
        print("  [speech_recognition not installed — voice transcription unavailable]")
        return ""
    except Exception as e:
        print(f"  [Transcription failed: {e}]")
        return ""
    finally:
        os.unlink(tmp.name)


def record_audio() -> np.ndarray:
    if not HAS_SOUNDDEVICE:
        print("  [sounddevice not installed — voice input unavailable]")
        return np.array([], dtype="int16")

    print("  [Recording... press Enter to stop]")
    audio_chunks = []
    stop_event = threading.Event()

    def callback(indata, frames, time_info, status):
        if not stop_event.is_set():
            audio_chunks.append(indata.copy())

    stream = sd.InputStream(
        samplerate=SAMPLE_RATE, channels=1, dtype="int16", callback=callback
    )
    stream.start()
    input()
    stop_event.set()
    stream.stop()
    stream.close()

    if not audio_chunks:
        return np.array([], dtype="int16")
    return np.concatenate(audio_chunks, axis=0)


def listen() -> str:
    audio = record_audio()
    if len(audio) == 0:
        return ""
    print("  [Transcribing...]")
    return transcribe_audio(audio)


# ── Task Progress ────────────────────────────────────────────────────────────

def show_progress(task_spec: dict):
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


# ── Main Loop ────────────────────────────────────────────────────────────────

def main():
    if not os.environ.get("GEMINI_API_KEY"):
        print("ERROR: Set GEMINI_API_KEY environment variable first.")
        print("  export GEMINI_API_KEY=<your-key>")
        sys.exit(1)

    voice_ok = HAS_SOUNDDEVICE and HAS_SOUNDFILE
    print("\nRobot Assistant (Ctrl+C to exit)")
    print("=" * 60)
    if voice_ok:
        print("  [Enter]  = Push-to-talk (press Enter to start, Enter to stop)")
    else:
        missing = []
        if not HAS_SOUNDDEVICE:
            missing.append("sounddevice")
        if not HAS_SOUNDFILE:
            missing.append("soundfile")
        print(f"  [Voice disabled — install: pip install {' '.join(missing)}]")
    print("  [type]   = Type a message and press Enter")
    print("=" * 60)

    history = []

    while True:
        try:
            raw = input("\nYou (type or press Enter to talk): ")
            command = raw.strip()

            if not command and voice_ok:
                command = listen()
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
                    if not confirm and voice_ok:
                        confirm = listen()
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
                    print("Robot: Executing task!")
                    print(json.dumps(task_spec, indent=2))
                    show_progress(task_spec)
                elif result2["type"] == "chat":
                    print(f"Robot: {result2['message']}")
                    speak(result2["message"])

                history.append({"user": confirm, "assistant": raw_json2})
                continue

            elif result["type"] == "command":
                task_spec = {k: v for k, v in result.items() if k != "type"}
                print("Robot: Executing task!")
                print(json.dumps(task_spec, indent=2))
                show_progress(task_spec)

            history.append({"user": command, "assistant": raw_json})

        except ValidationError as e:
            print(f"[VALIDATION ERROR] {e}", file=sys.stderr)
        except Exception as e:
            print(f"[ERROR] {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
