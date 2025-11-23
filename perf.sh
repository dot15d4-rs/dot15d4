ARGS="-q -p dot15d4-examples-nrf52840 --target thumbv7em-none-eabihf --release"
SUT="$ARGS --bin radio"
ASYNC_BLOCK_REGEX="[0-9]+ \{async block@examples/nrf52840/src/bin/radio/main.rs"

# Save current branch name
BRANCH=$(git rev-parse --abbrev-ref HEAD)

# Get current commit
CURRENT_COMMIT=$(git rev-parse HEAD)

# Get previous commit
PREV_COMMIT=$(git rev-parse HEAD~1)

# Run on previous commit
git checkout -q "$PREV_COMMIT"
FLASH_SIZE_BEFORE=$(cargo size $SUT 2>&1)
ASYNC_BLOCK_SIZE_BEFORE=$(cargo clean $ARGS && cargo +nightly rustc $SUT -- -Zprint-type-sizes | top-type-sizes | egrep --color=never "$ASYNC_BLOCK_REGEX")

# Run on current commit
git checkout -q "$CURRENT_COMMIT"
FLASH_SIZE_AFTER=$(cargo size $SUT 2>&1)
ASYNC_BLOCK_SIZE_AFTER=$(cargo clean $ARGS && cargo +nightly rustc $SUT -- -Zprint-type-sizes | top-type-sizes | egrep --color=never "$ASYNC_BLOCK_REGEX")

# Return to the original branch
git checkout "$BRANCH"

echo Flash Size
echo ==========
echo "Before\n$FLASH_SIZE_BEFORE"
echo "After\n$FLASH_SIZE_AFTER"

echo
echo Main Async Block
echo ================
echo "Before\n$ASYNC_BLOCK_SIZE_BEFORE"
echo "After\n$ASYNC_BLOCK_SIZE_AFTER"
