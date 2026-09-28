"""Entry point for `flet run` / `flet build` (see [tool.flet] in pyproject.toml):
the MCDS Trace UI.  Arguments after `--` go to it, e.g.
`flet run -- --elf app.elf`."""

import sys

from mcds_trace.ui.app import main

if __name__ == '__main__':          # not in the spawned worker processes
    sys.exit(main())
