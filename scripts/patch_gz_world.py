#!/usr/bin/env python3
"""Inject missing Gazebo system plugins into a world file.

Usage: patch_gz_world.py <world_file>
Prints the path to a patched copy in /tmp (or the original if no changes needed).
"""
import re
import sys
import hashlib
import tempfile
from pathlib import Path

PLUGINS = [
    ('gz-sim-physics-system',
     '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'),
    ('gz-sim-user-commands-system',
     '<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>'),
    ('gz-sim-scene-broadcaster-system',
     '<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>'),
    ('gz-sim-sensors-system',
     '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
     '<render_engine>ogre2</render_engine></plugin>'),
    ('gz-sim-imu-system',
     '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'),
    ('gz-sim-magnetometer-system',
     '<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>'),
]

# Gazebo Harmonic reserves the world name "world"
RESERVED_WORLD_NAME = 'world'
REPLACEMENT_WORLD_NAME = 'picar2_sim'


def patch(content: str) -> tuple[str, bool]:
    changed = False

    # Rename reserved world name
    def rename_world(m):
        nonlocal changed
        result = re.sub(
            r'(<world\s+name\s*=\s*["\'])' + RESERVED_WORLD_NAME + r'(["\'])',
            lambda mm: mm.group(1) + REPLACEMENT_WORLD_NAME + mm.group(2),
            m.group(0),
        )
        if result != m.group(0):
            changed = True
        return result

    content = re.sub(r'<world\b[^>]*>', rename_world, content, count=1)

    # Inject missing plugins after the <world ...> opening tag
    missing = '\n    '.join(
        xml for key, xml in PLUGINS if key not in content
    )
    if missing:
        content = re.sub(
            r'(<world\b[^>]*>)',
            r'\1\n    ' + missing + '\n',
            content,
            count=1,
        )
        changed = True

    return content, changed


def main():
    if len(sys.argv) != 2:
        print(f'Usage: {sys.argv[0]} <world_file>', file=sys.stderr)
        sys.exit(1)

    src = Path(sys.argv[1])
    if not src.exists():
        print(src, end='')  # pass through; gz will report the error
        return

    content = src.read_text(errors='replace')
    patched, changed = patch(content)

    if not changed:
        print(src, end='')
        return

    # Stable temp path so repeated make runs reuse it
    digest = hashlib.md5(str(src).encode()).hexdigest()[:8]
    dst = Path(tempfile.gettempdir()) / f'gz_patched_{digest}_{src.name}'
    dst.write_text(patched)
    print(dst, end='')


if __name__ == '__main__':
    main()
