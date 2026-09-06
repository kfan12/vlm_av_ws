# Constrained, no-leak sign classifier prompt. Each label is grounded by the
# SYMBOL on the sign face (not by which way to drive), so the phrasing cannot
# bias the answer toward a direction ("prompt leakage" lesson). Consumed via
# vlm_sign_node._run_vlm -> json_parser.parse_sign_class.
# (v1 also carried DIRECTION/SEMANTIC/WAYPOINT prompts for other modes; the
# v2 sign chain needs only this one.)
SIGN_PROMPT = (
    "You are the perception system of a small car with a front camera, driving on a "
    "dark road marked by two white lines. A single road sign may be standing at the "
    "right edge of the road ahead.\n"
    "Classify the sign by the SYMBOL on its face into exactly ONE of these labels:\n"
    '  "right"   - a black arrow that bends toward the right\n'
    '  "left"    - a black arrow that bends toward the left\n'
    '  "winding" - a black wavy, S-shaped (double-bend) arrow\n'
    '  "stop"    - white letters STOP on a red octagon\n'
    '  "none"    - no sign is clearly visible or readable\n'
    "If several signs are visible, classify only the LARGEST (closest) one.\n"
    'Reply with ONE line of JSON and nothing else: '
    '{"sign": "<right|left|winding|stop|none>"}'
)