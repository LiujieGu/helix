# AGENTS.md

C++20 optimization-algorithm library (`helix`). CMake build, VS Code + CMake Tools.

## Build / test / run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # configure
cmake --build build -j                            # build
./build/helix                                    # run
ctest --test-dir build --output-on-failure        # run all tests
```

- CMake is installed at `~/.local/bin` (no sudo). It is on PATH via `~/.bashrc`; if a fresh non-login shell lacks it, use the full path `~/.local/bin/cmake`.
- Default build type is `Release` (set in `CMakeLists.txt` when unset). Use `-DCMAKE_BUILD_TYPE=Debug` for gdb.
- `compile_commands.json` is generated into `build/` and consumed by VS Code IntelliSense (`.vscode/settings.json`).

## Layout

- `src/main.cpp` — program entrypoint, added via `add_executable(helix ...)`.
- `tests/` — each test is `add_executable` + `add_test` in `tests/CMakeLists.txt`; wired into top-level `ctest`.
- `build/` — generated, gitignored.

## Conventions

- C++20, extensions off, `-Wall -Wextra -Wpedantic`.
- Code style defined by `.clang-format` (Google-based, 4-space, 100 col). Format: `clang-format -i <files>`.
- Project name `helix` is hardcoded in `CMakeLists.txt` (`project` + `add_executable`). When renaming, replace both.

## Notes

- No third-party deps, no package manager, no CI yet.
- Bootstrapped from a generic C++ template; now a named library.
