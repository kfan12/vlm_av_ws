import json
import re

SIGN_LABELS = ("right", "left", "winding", "stop", "none")


def parse_sign_class(response: str, valid_labels=SIGN_LABELS) -> dict | None:
    """Parse a sign-class VLM response -> {'action': label, 'confidence': float|None}.

    Prefer the JSON object ({"sign":..., "confidence":...}); fall back to keyword
    search over the raw text. Returns None on an unknown/empty/garbled answer so the
    caller FAILS SAFE (no invented class). 'none' is a valid label (no sign in
    view), returned as a dict; None means the read itself failed."""
    conf = None
    d = parse_vlm_json(response)
    if isinstance(d, dict):
        s = str(d.get('sign', d.get('action', ''))).strip().lower()
        c = d.get('confidence', None)
        if isinstance(c, (int, float)):
            conf = float(c)
        if s in valid_labels:
            return {'action': s, 'confidence': conf}

    low = response.lower()
    if conf is None:
        m = re.search(r"confidence['\"]?\s*[:=]\s*([01](?:\.\d+)?)", low)
        if m:
            conf = float(m.group(1))

    # Negation first: "no sign", "none", "cannot read" -> 'none', UNLESS a
    # concrete glyph word is also present (Qwen sometimes says "a left arrow,
    # no other signs"). Without this, "no stop sign visible" matched "stop".
    neg = bool(re.search(r"\bno\b[\w ]{0,24}\bsign", low)) or any(p in low for p in (
        "none", "no readable", "not visible", "cannot identify",
        "can't identify", "nothing"))
    glyph = any(p in low for p in (
        "arrow", "octagon", "winding", "wavy", "s-shape", "s-curve", "diamond"))
    if neg and not glyph and "none" in valid_labels:
        return {'action': 'none', 'confidence': conf}

    # priority: explicit glyph words > directional words (so "winding"/"stop"
    # win over a stray "right"/"left" mentioned in passing).
    for kw, lab in (("winding", "winding"), ("wavy", "winding"), ("s-shape", "winding"),
                    ("s-curve", "winding"), ("double", "winding"),
                    ("octagon", "stop"), ("stop", "stop"),
                    ("no sign", "none"), ("none", "none"),
                    ("right", "right"), ("left", "left")):
        if kw in low and lab in valid_labels:
            return {'action': lab, 'confidence': conf}

    # Colour fallback: stop is the ONLY red sign, so a red sign face with no
    # arrow read is almost certainly it (the STOP lettering / octagon shape is
    # the first thing to blur out at range). Yellow is NOT a fallback -- all
    # three warning signs are yellow.
    if "stop" in valid_labels and "red" in low and "arrow" not in low:
        return {'action': 'stop', 'confidence': conf}
    return None


def parse_vlm_json(response: str) -> dict | None:
    """Extract JSON from VLM response string."""
    try:
        return json.loads(response)
    except json.JSONDecodeError:
        pass
    match = re.search(r'\{.*\}', response, re.DOTALL)
    if match:
        try:
            return json.loads(match.group())
        except json.JSONDecodeError:
            return None
    return None
