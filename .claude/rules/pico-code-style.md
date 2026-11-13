---
paths:
  - "driver/**/*.py"
  - "tests/bench/**/*.py"
  - "tools/esc_buddy/**/*.py"
---

# Naming and style of the MicroPython code

The code that runs on a Pico (the driver, the bench code, the ESC buddy's board code) is MicroPython on a microcontroller, not a published CPython package. These rules keep it plain.

- **No `_` prefix for visibility.** Attributes and methods are named plainly (`self.motors`, `self.state`, `runner.loop()`). The underscore convention buys nothing here and only adds noise. Not affected: `_thread` (a stdlib module), `__init__`, `for _ in range(n)` for a throwaway loop variable, and `_NAME = const(...)` in library code copied from micropython-lib (`tests/bench/session_recorder/sdcard.py`), where the underscore is MicroPython's own idiom for a constant kept out of the module's globals.
- **snake_case** throughout `driver/`, including method names.
- **Plain attributes over accessor methods.** Keep a method only when it does real work: `get_all_throttles()` converts an `array` to a list, `is_armed()` compares against a state constant.
- **No f-strings on error paths in `driver/`.** They allocate, and that code may run on a core with a constrained stack.
