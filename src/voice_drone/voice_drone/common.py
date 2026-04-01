import json
import math
import os
import queue
import re
import tempfile
import time
import wave
from typing import Any, Dict, List, Optional

try:
    import numpy as np
except Exception:
    np = None

try:
    import requests
except Exception:
    requests = None


WORD_NUMS = {
    'zero': 0.0,
    'one': 1.0,
    'two': 2.0,
    'three': 3.0,
    'four': 4.0,
    'five': 5.0,
    'six': 6.0,
    'seven': 7.0,
    'eight': 8.0,
    'nine': 9.0,
    'ten': 10.0,
}


TARGET_MAP = {
    'coke can': ['coke can', 'coca cola can', 'coca-cola can', 'cola can', 'soda can', 'can', 'bottle', 'cup'],
    'coca cola can': ['coke can', 'coca cola can', 'coca-cola can', 'cola can', 'soda can', 'can', 'bottle', 'cup'],
    'soda can': ['soda can', 'coke can', 'coca cola can', 'coca-cola can', 'cola can', 'can', 'bottle', 'cup'],
    'bottle': ['bottle'],
    'cup': ['cup'],
    'person': ['person', 'human'],
    'chair': ['chair'],
    'car': ['car'],
}


INTENT_SCHEMA: Dict[str, Any] = {
    'type': 'object',
    'additionalProperties': False,
    'properties': {
        'ok': {'type': 'boolean'},
        'action': {
            'type': 'string',
            'enum': [
                'none',
                'arm_takeoff',
                'land',
                'hold',
                'go_to_object',
                'move_body',
                'rotate',
                'disarm',
                'cancel',
                'status',
            ],
        },
        'target': {'type': 'string'},
        'alt_m': {'type': 'number'},
        'dist_m': {'type': 'number'},
        'yaw_deg': {'type': 'number'},
        'move': {
            'type': 'object',
            'additionalProperties': False,
            'properties': {
                'x_m': {'type': 'number'},
                'y_m': {'type': 'number'},
                'z_m': {'type': 'number'},
            },
            'required': ['x_m', 'y_m', 'z_m'],
        },
        'raw': {'type': 'string'},
    },
    'required': ['ok', 'action', 'target', 'alt_m', 'dist_m', 'yaw_deg', 'move', 'raw'],
}


DEFAULT_INTENT = {
    'ok': True,
    'action': 'none',
    'target': '',
    'alt_m': 0.0,
    'dist_m': 0.0,
    'yaw_deg': 0.0,
    'move': {'x_m': 0.0, 'y_m': 0.0, 'z_m': 0.0},
    'raw': '',
}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def now_s() -> float:
    return time.time()


def jdump(obj: Any) -> str:
    return json.dumps(obj, separators=(',', ':'), ensure_ascii=False)


def jload(s: str, fallback: Optional[Any] = None) -> Any:
    try:
        return json.loads(s)
    except Exception:
        return fallback


def get_env(name: str, default: str = '') -> str:
    return os.environ.get(name, default)


def get_bool_env(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.lower() in ['1', 'true', 'yes', 'on']


def first_num(s: str, default: Optional[float] = None) -> Optional[float]:
    m = re.search(r'(-?\d+(?:\.\d+)?)', s)
    if m:
        try:
            return float(m.group(1))
        except Exception:
            return default

    for k, v in WORD_NUMS.items():
        if re.search(r'\b' + re.escape(k) + r'\b', s):
            return v

    return default


def clean_text(s: str) -> str:
    return re.sub(r'\s+', ' ', s.strip())


def clean_target(s: str) -> str:
    s = s.lower().strip()
    s = re.sub(r'^[\s:,-]+', '', s)
    s = re.sub(r'[\s\.\?!,]+$', '', s)
    return s


def _build_alias_map() -> Dict[str, str]:
    alias_map: Dict[str, str] = {}

    for canonical in TARGET_MAP.keys():
        alias_map[clean_target(canonical)] = canonical

    for canonical, aliases in TARGET_MAP.items():
        for alias in aliases:
            alias_key = clean_target(alias)
            if not alias_key:
                continue
            alias_map.setdefault(alias_key, canonical)

    return alias_map


TARGET_ALIAS_MAP = _build_alias_map()


def canon_target(s: str) -> str:
    t = clean_target(s)
    if not t:
        return ''
    return TARGET_ALIAS_MAP.get(t, t)


def target_aliases(target: str) -> List[str]:
    t = canon_target(target)
    vals = TARGET_MAP.get(t, [t])
    out = []
    for x in [t] + list(vals):
        x = x.lower().strip()
        if x and x not in out:
            out.append(x)
    return out


def label_match(label: str, target: str) -> bool:
    lab = label.lower().strip()
    for a in target_aliases(target):
        if lab == a:
            return True
        if a in lab or lab in a:
            return True
    return False


def make_intent(raw: str) -> Dict[str, Any]:
    out = dict(DEFAULT_INTENT)
    out['move'] = dict(DEFAULT_INTENT['move'])
    out['raw'] = raw
    return out


def _after_one_of(txt: str, keys: List[str]) -> str:
    low = txt.lower()
    for k in keys:
        i = low.find(k)
        if i >= 0:
            return txt[i + len(k):].strip()
    return ''


def fallback_parse_intent(raw: str, default_takeoff_alt_m: float = 1.5) -> Dict[str, Any]:
    txt = clean_text(raw)
    low = txt.lower()
    out = make_intent(raw)

    if not txt:
        return out

    if any(x in low for x in ['status', 'what are you doing', 'what is happening', 'state']):
        out['action'] = 'status'
        return out

    if any(x in low for x in ['cancel', 'never mind', 'nevermind', 'forget it']):
        out['action'] = 'cancel'
        return out

    if 'disarm' in low:
        out['action'] = 'disarm'
        return out

    if 'land' in low:
        out['action'] = 'land'
        return out

    if any(x in low for x in ['stop', 'hold', 'hover', 'freeze']):
        out['action'] = 'hold'
        return out

    if ('arm' in low and ('takeoff' in low or 'take off' in low)) or 'takeoff' in low or 'take off' in low:
        out['action'] = 'arm_takeoff'
        out['alt_m'] = first_num(low, default_takeoff_alt_m) or default_takeoff_alt_m
        return out

    if any(x in low for x in ['go to', 'find ', 'look for', 'track ', 'approach ']):
        out['action'] = 'go_to_object'
        tgt = _after_one_of(txt, ['go to', 'find', 'look for', 'track', 'approach'])
        out['target'] = canon_target(tgt)
        return out

    if any(x in low for x in ['turn ', 'rotate ', 'yaw ']):
        out['action'] = 'rotate'
        deg = first_num(low, 30.0) or 30.0
        if 'left' in low or 'counterclockwise' in low or 'counter-clockwise' in low:
            deg = -abs(deg)
        elif 'right' in low or 'clockwise' in low:
            deg = abs(deg)
        out['yaw_deg'] = deg
        return out

    dirs = ['forward', 'back', 'backward', 'left', 'right', 'up', 'down']
    if any(x in low for x in dirs):
        out['action'] = 'move_body'
        d = first_num(low, 1.0) or 1.0
        mv = {'x_m': 0.0, 'y_m': 0.0, 'z_m': 0.0}
        if 'forward' in low:
            mv['x_m'] += abs(d)
        if 'backward' in low or re.search(r'\bback\b', low):
            mv['x_m'] -= abs(d)
        if 'left' in low:
            mv['y_m'] += abs(d)
        if 'right' in low:
            mv['y_m'] -= abs(d)
        if 'up' in low:
            mv['z_m'] += abs(d)
        if 'down' in low:
            mv['z_m'] -= abs(d)
        out['move'] = mv
        out['dist_m'] = abs(d)
        return out

    return out


def openai_headers(api_key: str) -> Dict[str, str]:
    return {
        'Authorization': f'Bearer {api_key}',
    }


def extract_output_text(resp_json: Dict[str, Any]) -> str:
    if isinstance(resp_json.get('output_text'), str):
        return resp_json['output_text']

    for item in resp_json.get('output', []):
        if item.get('type') != 'message':
            continue
        for c in item.get('content', []):
            if c.get('type') == 'output_text':
                return c.get('text', '')
    return ''


def call_openai_json(
    raw_text: str,
    api_key: str,
    model: str,
    base_url: str,
    schema: Dict[str, Any],
    instructions: str,
    timeout_sec: float = 40.0,
) -> Optional[Dict[str, Any]]:
    if requests is None:
        return None
    if not api_key:
        return None

    url = base_url.rstrip('/') + '/responses'
    body = {
        'model': model,
        'instructions': instructions,
        'input': raw_text,
        'max_output_tokens': 220,
        'tool_choice': 'none',
        'text': {
            'format': {
                'type': 'json_schema',
                'name': 'drone_intent',
                'schema': schema,
            }
        },
    }

    r = requests.post(
        url,
        headers={
            **openai_headers(api_key),
            'Content-Type': 'application/json',
        },
        data=json.dumps(body),
        timeout=timeout_sec,
    )
    r.raise_for_status()
    data = r.json()
    txt = extract_output_text(data)
    if not txt:
        return None
    obj = json.loads(txt)
    return obj


def call_openai_transcribe_file(
    wav_path: str,
    api_key: str,
    model: str,
    base_url: str,
    timeout_sec: float = 60.0,
) -> Optional[str]:
    if requests is None:
        return None
    if not api_key:
        return None

    url = base_url.rstrip('/') + '/audio/transcriptions'
    with open(wav_path, 'rb') as f:
        files = {
            'file': (os.path.basename(wav_path), f, 'audio/wav'),
        }
        data = {
            'model': model,
        }
        r = requests.post(
            url,
            headers=openai_headers(api_key),
            files=files,
            data=data,
            timeout=timeout_sec,
        )
        r.raise_for_status()
        obj = r.json()

    if isinstance(obj, dict):
        txt = obj.get('text')
        if isinstance(txt, str):
            return txt.strip()
    return None


def save_wav_int16(path: str, audio_arr: Any, sample_rate: int, channels: int) -> None:
    if np is None:
        raise RuntimeError('numpy is not installed')

    arr = np.asarray(audio_arr)
    if arr.ndim == 1 and channels == 1:
        pass
    elif arr.ndim == 2 and arr.shape[1] == channels:
        pass
    else:
        arr = arr.reshape(-1)

    arr = np.clip(arr, -1.0, 1.0)
    arr_i16 = (arr * 32767.0).astype(np.int16)

    with wave.open(path, 'wb') as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(arr_i16.tobytes())


def tmp_wav_path(prefix: str = 'voice_drone_') -> str:
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.wav')
    os.close(fd)
    return path
