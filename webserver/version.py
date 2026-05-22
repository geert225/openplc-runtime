"""Runtime version exposed at /api/version and via the
X-OpenPLC-Runtime-Version response header.

The string is injected at image build time via the RUNTIME_VERSION
build-arg (mirrors what strucpp, openplc-editor, and the other
projects do — version comes from the GitHub tag).  Falls back to
``dev`` when running outside CI / from a local source tree so the
restapi keeps working during development.

Editors consume this to decide whether they're allowed to push
firmware: the v4.1.x runtime ships the STruC++ pipeline, which is
not wire-compatible with the MatIEC pipeline 4.0.x runtimes shipped.
"""

import os

RUNTIME_VERSION = os.getenv("RUNTIME_VERSION", "dev")
