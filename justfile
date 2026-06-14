# gnss-rcv task runner — `just <recipe>` (run `just` alone to list).
# Canonical commands for the build/test/run/validate workflow, so humans, CI, and
# agents all invoke the exact same thing (and a single `Bash(just:*)` allow-list
# entry covers them all).

# Default: list the recipes.
default:
    @just --list

# Full local gate — the three CI checks, in order. Run before every commit.
check: fmt-check lint test

# Format the tree.
fmt:
    cargo fmt --all

# Verify formatting (non-mutating; what CI runs).
fmt-check:
    cargo fmt --all -- --check

# Clippy with warnings denied (all targets).
lint:
    cargo clippy --release --all-targets -- -D warnings

# Unit + fast integration tests.
test:
    cargo test --release

# Also run the heavy #[ignore]'d end-to-end tests (needs the recordings).
test-all:
    cargo test --release -- --include-ignored

# Release build.
build:
    cargo build --release

# Validate GPS fix + Galileo I/NAV decode vs known recordings (skips if absent).
validate:
    ./scripts/validate_fix.py

# Quick fix smoke-test across every present recording (seek past warmup, run a
# short --exit-on-fix window, check a fix at the right site). `just smoke ifen`
# filters by name; pass-through args, e.g. `just smoke "-j 4"`.
smoke *args:
    ./scripts/smoke_fix.py {{args}}

# Examples:
#   just run resources/gpssim_2xi16 "-t 2xi16 -x"
#   just run resources/L1_20211226_082212_12MHz_I.bin "-t i8 --fs 12M --fi 3M --sig E1B"

# Run the receiver on a recording: just run <file> "<flags>".
run recording args="":
    cargo run --release -- -f {{recording}} {{args}}

# Galileo E1-B I/NAV decode demo (PocketSDR capture, ~3 s).
galileo:
    cargo run --release -- -f resources/L1_20211226_082212_12MHz_I.bin \
        -t i8 --fs 12M --fi 3M --sig E1B --num-msec 3000

# List / fetch the IQ sample recordings (passes args through to fetch.py).
fetch *args:
    ./resources/fetch.py {{args}}

# Synthetic bit-sync bench at a chosen C/N0, e.g. `just bench 40`.
bench cn0="45":
    RUST_LOG=info cargo run --release --example bitsync_bench -- {{cn0}}
