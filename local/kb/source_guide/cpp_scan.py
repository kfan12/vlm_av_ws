"""cpp_scan.py - scope-aware C++ function/class finder (no libclang): strips comments and
strings, tracks braces, and records every function body opened at namespace or class scope."""
import re

KW = {'if', 'for', 'while', 'switch', 'catch', 'return', 'sizeof', 'do', 'else', 'try'}


def strip(src):
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith('//', i):
            j = src.find('\n', i)
            j = n if j < 0 else j
            out.append(' ' * (j - i)); i = j
        elif src.startswith('/*', i):
            j = src.find('*/', i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r'[^\n]', ' ', src[i:j])); i = j
        elif c == '"' or c == "'":
            if c == '"' and src[i - 1:i] == 'R' and src.startswith('R"(', i - 1):
                j = src.find(')"', i)
                j = n if j < 0 else j + 2
            else:
                j = i + 1
                while j < n and src[j] != c:
                    j += 2 if src[j] == '\\' else 1
                j += 1
            out.append(c + re.sub(r'[^\n]', ' ', src[i + 1:j - 1]) + c if j - i >= 2 else src[i:j]); i = j
        elif c == '#' and (i == 0 or src[i - 1] == '\n'):
            j = src.find('\n', i)
            j = n if j < 0 else j
            out.append(' ' * (j - i)); i = j
        else:
            out.append(c); i += 1
    return ''.join(out)


def comment_above(lines, idx):
    out, j = [], idx - 1
    if j >= 1 and not lines[j].strip() and lines[j - 1].strip().startswith(('//', '*', '/*')):
        j -= 1  # one blank line between a comment block and its function still belongs to it
    while j >= 0 and lines[j].strip().startswith(('//', '*', '/*')):
        s = re.sub(r'^(///?|/\*+|\*+/?)\s?', '', lines[j].strip()).rstrip('*/ ').strip()
        out.insert(0, s); j -= 1
    txt = ' '.join(x for x in out if x and not re.fullmatch(r'-+\s*\w*\s*-*', x))
    return txt


def scan(path):
    src = open(path).read()
    s = strip(src)
    lines = src.split('\n')
    line_of = lambda pos: s.count('\n', 0, pos)  # 0-based
    items = []
    stack = []  # (kind, record or None)
    boundary = 0
    i = 0
    while i < len(s):
        c = s[i]
        if c == '{':
            header = s[boundary:i]
            hs = ' '.join(header.split())
            parent = stack[-1][0] if stack else 'ns'
            kind, rec = 'other', None
            if parent in ('ns', 'class'):
                if re.search(r'\bnamespace\b', hs) or hs.startswith('extern'):
                    kind = 'ns'
                elif re.match(r'^(template\s*<.*>\s*)?(class|struct|union)\s+\w+', hs) and '(' not in hs.split(':')[0]:
                    kind = 'class'
                    m = re.match(r'^(?:template\s*<.*>\s*)?(class|struct|union)\s+(\w+)', hs)
                    start = boundary + len(header) - len(header.lstrip())
                    ln = line_of(start)
                    rec = {'kind': m.group(1), 'name': m.group(2), 'line': ln + 1, 'sig': hs, 'doc': comment_above(lines, ln)}
                elif re.match(r'^(enum)\b', hs):
                    kind = 'other'
                elif '(' in hs and not re.search(r'=\s*\[', hs) and not hs.startswith('='):
                    # name = identifier right before the first '(' (constructor init lists come after ')')
                    m = re.match(r'^(.*?)([~\w]+(?:::[~\w]+)*)\s*\(', hs)
                    if m and m.group(2).split('::')[-1] not in KW and not re.search(r'=\s*$', m.group(1)):
                        kind = 'fn'
                        start = boundary + len(header) - len(header.lstrip())
                        ln = line_of(start)
                        # signature without the init list and trailing qualifiers
                        depth, k = 0, hs.index('(', len(m.group(1)))
                        for k2 in range(k, len(hs)):
                            depth += hs[k2] == '('
                            depth -= hs[k2] == ')'
                            if depth == 0:
                                break
                        sig = hs[:k2 + 1]
                        tail = hs[k2 + 1:].strip()
                        q = re.match(r'^((?:const|override|noexcept|final|\s)*)', tail).group(1).strip()
                        if q:
                            sig += ' ' + q
                        sig = re.sub(r'^(?:public|private|protected)\s*:\s*', '', sig)
                        name = m.group(2)
                        if parent == 'class' and stack and stack[-1][1] and '::' not in name:
                            name = stack[-1][1]['name'] + '::' + name
                        rec = {'kind': 'fn', 'name': name, 'line': ln + 1, 'sig': sig, 'doc': comment_above(lines, ln)}
            if rec:
                items.append(rec)
            stack.append((kind, rec))
            boundary = i + 1
        elif c == '}':
            if stack:
                kind, rec = stack.pop()
                if rec:
                    rec['end'] = line_of(i) + 1
            boundary = i + 1
        elif c == ';' or (c == ':' and re.search(r'\b(public|private|protected)\s*$', s[boundary:i])):
            if not stack or stack[-1][0] in ('ns', 'class'):
                boundary = i + 1
        i += 1
    for it in items:
        it['sig'] = re.sub(r'^(?:public|private|protected)\s*:\s*', '', it['sig'])
    return items
