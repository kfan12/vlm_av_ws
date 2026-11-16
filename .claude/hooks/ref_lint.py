#!/usr/bin/env python3
"""ref_lint.py - everything a skill, an agent or a CLAUDE.md names must exist.

  python3 .claude/hooks/ref_lint.py        exit 1 with file:line per broken reference
  python3 .claude/hooks/ref_lint.py -v     also list what was checked

Reads every `backticked` token in .claude/skills/*/SKILL.md, .claude/agents/*.md
and every CLAUDE.md of the workspace, and checks the ones it can classify:
  a path (has a slash, or a file name with a known extension)  -> exists in the tree
  a ROS topic (starts with /)                                  -> occurs in the sources
  a launch argument (name:=value)                              -> declared by a launch file
It also flags a line that says something "does not exist yet", is "planned" or
comes "once written" when the path it names exists: the stale claim that made
the first four skills wrong.
Tokens with placeholders (<x>, *, $, {..}, |) are skipped, as are files a run
produces (ref_lint_allow.txt lists those names, one per line).
A reference that is right but cannot be resolved goes into ref_lint_allow.txt.
"""
import glob
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HERE = os.path.dirname(os.path.abspath(__file__))
SKIP_DIRS = {'.git', 'build', 'install', 'log', 'data', '__pycache__', 'worktrees', 'node_modules'}
FILE_EXT = ('.md', '.py', '.sh', '.cpp', '.hpp', '.h', '.yaml', '.yml', '.json', '.msg', '.xacro', '.rviz',
            '.html', '.pdf', '.txt', '.ps1', '.csv', '.png', '.jsonl', '.log', '.sdf', '.xml')
SOURCE_EXT = ('.cpp', '.hpp', '.h', '.py', '.yaml', '.yml', '.xacro', '.sh', '.sdf', '.rviz')
# absolute paths outside the workspace are not this lint's business
OUTSIDE = ('/home/', '/tmp', '/opt/', '/mnt/', '/dev/', '/usr/', '/etc/', '~', '\\\\', '//')
PLACEHOLDER = re.compile(r'[<>*${}|]|\.\.\.|\bN\b|YYYY|NN')
RANGE = re.compile(r'\d-\d+\.|[a-z]\d-[a-z]\d\.')  # day05-07.md, l0-l5.md: several files in one name
STALE = re.compile(r'once (it|they) exists?|once written|not (yet )?built( yet)?\b|does not exist( yet)?|not yet written|\bplanned `', re.I)
# a stale claim is only about code and results, not about a README or a CLAUDE.md a sentence happens to name
STALE_ABOUT = ('src/', 'scripts/', 'tools/', 'docs/reports/', 'docs/baselines/')
# top-level directories the tree walk skips but documents may name
TOP_DIRS = {'install', 'build', 'log', 'data'}
TOKEN = re.compile(r'`([^`\n]+)`')


def tree():
    paths, names = set(), set()
    for root, dirs, files in os.walk(WS):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        rel = os.path.relpath(root, WS)
        for d in dirs:
            paths.add(os.path.normpath(os.path.join(rel, d)))
            names.add(d)
        for f in files:
            paths.add(os.path.normpath(os.path.join(rel, f)))
            names.add(f)
    return paths, names


def sources():
    chunks = []
    for top in ('src', 'scripts', 'tools'):
        for root, dirs, files in os.walk(os.path.join(WS, top)):
            dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
            for f in files:
                if f.endswith(SOURCE_EXT):
                    try:
                        with open(os.path.join(root, f), encoding='utf-8') as fh:
                            chunks.append(fh.read())
                    except (OSError, UnicodeDecodeError):
                        pass
    return '\n'.join(chunks)


def launch_args():
    args = set()
    for path in glob.glob(os.path.join(WS, 'src', '*', 'launch', '*.py')):
        args.update(re.findall(r"DeclareLaunchArgument\(\s*'([a-z_]+)'", open(path).read()))
    for path in glob.glob(os.path.join(WS, 'src', '*', 'urdf', '*.xacro')):
        args.update(re.findall(r'<xacro:arg name="([a-z_]+)"', open(path).read()))
    return args


def docs():
    out = sorted(glob.glob(os.path.join(WS, '.claude', 'skills', '*', 'SKILL.md')))
    out += sorted(glob.glob(os.path.join(WS, '.claude', 'agents', '*.md')))
    for root, dirs, files in os.walk(WS):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS and d != 'local']
        if 'CLAUDE.md' in files:
            out.append(os.path.join(root, 'CLAUDE.md'))
    return out


class Lint:
    def __init__(self):
        self.paths, self.names = tree()
        self.src = sources()
        self.args = launch_args()
        allow = os.path.join(HERE, 'ref_lint_allow.txt')
        self.allow = {ln.strip() for ln in open(allow) if ln.strip() and not ln.startswith('#')} if os.path.exists(allow) else set()
        self.checked = 0

    def path_exists(self, word, doc_dir):
        w = os.path.normpath(word.split('#')[0].rstrip('/'))
        w = re.sub(r':\d+$', '', w)
        if w.split('/')[0] in TOP_DIRS:
            return True  # build and run output: absent on a clean checkout, and never checked
        if w in self.paths or os.path.normpath(os.path.join(doc_dir, w)) in self.paths:
            return True
        return any(p.endswith('/' + w) for p in self.paths)

    def check_word(self, word, doc_dir):
        """None when fine or not classifiable, else the reason it is broken."""
        if PLACEHOLDER.search(word):  # before any stripping: "..." is a placeholder, not punctuation
            return None
        # trailing punctuation only: a leading dot belongs to the path (.claude/...)
        word = word.lstrip('(["\'').rstrip('.,;:)]"\'')
        word = word.split('::')[0]  # file.hpp::function
        if not word or word in self.allow or PLACEHOLDER.search(word) or RANGE.search(word) or '_msgs/' in word:
            return None
        m = re.fullmatch(r'([a-z_]+):=\S*', word)
        if m:
            self.checked += 1
            return None if m.group(1) in self.args else f'launch argument `{m.group(1)}` is declared by no launch file'
        if word.startswith(OUTSIDE) or re.match(r'^[A-Za-z]:', word) or '..' in word:
            return None
        if word.startswith('/'):
            if re.search(r'\.[a-z0-9]+$', word) and not word.endswith(('.x', '.z')):
                return None  # an absolute file path somewhere else
            self.checked += 1
            return None if word.rstrip('/') in self.src else f'topic `{word}` occurs in no source file'
        if '/' in word:
            if not re.fullmatch(r'[\w./#:+-]+', word) or re.fullmatch(r'[\w]+/msg/\w+', word) or re.fullmatch(r'\w+/\w+', word) and not word.endswith(FILE_EXT) and word.split('/')[0] not in self.names:
                return None  # a message type, a ratio, a pair of words
            self.checked += 1
            return None if self.path_exists(word, doc_dir) else f'path `{word}` does not exist'
        if word.endswith(FILE_EXT) and re.fullmatch(r'[\w.+-]+', word):
            self.checked += 1
            return None if word in self.names else f'file `{word}` exists nowhere in the tree'
        return None

    def run(self, verbose=False):
        problems = []
        for doc in docs():
            rel = os.path.relpath(doc, WS)
            doc_dir = os.path.dirname(rel)
            in_code = False
            for n, line in enumerate(open(doc, encoding='utf-8').read().splitlines(), 1):
                if line.lstrip().startswith('```'):
                    in_code = not in_code
                    continue
                if in_code:
                    continue
                tokens = TOKEN.findall(line)
                for tok in tokens:
                    for word in tok.split():
                        why = self.check_word(word, doc_dir)
                        if why:
                            problems.append(f'{rel}:{n}: {why}')
                stale = STALE.search(line)
                # a quoted phrase is a search term or an example, not a claim
                if stale and line[max(0, stale.start() - 1)] != '"':
                    existing = [w for tok in tokens for w in tok.split()
                                if w.startswith(STALE_ABOUT) and w.rstrip('/') + '/' not in STALE_ABOUT
                                and not PLACEHOLDER.search(w)
                                and self.path_exists(w.rstrip('.,;:)'), doc_dir)]
                    if existing:
                        problems.append(f'{rel}:{n}: says "{STALE.search(line).group(0)}" but `{existing[0]}` exists '
                                        '(say where to check instead of what exists)')
        if verbose:
            print(f'{len(docs())} documents, {self.checked} references checked')
        return problems


def main():
    lint = Lint()
    problems = lint.run('-v' in sys.argv)
    if problems:
        print(f'ref lint: {len(problems)} broken reference(s)')
        print('\n'.join('  ' + p for p in problems))
        return 1
    print(f'ref lint: {lint.checked} references in {len(docs())} documents resolve')
    return 0


if __name__ == '__main__':
    sys.exit(main())
