# g1_msgs - agent guide

Interface-only package: the shared msg/srv contracts for the whole humis-g1
stack. No runtime code. Every other g1_* package depends on these types, so a
change here can ripple widely; keep definitions minimal and stable.

## Invariants (do not break)
- **Builds first.** g1_msgs is a dependency of every other package; message
  generation must succeed before downstream packages configure.
- **Add, do not repurpose.** Renaming/retyping/reordering an existing field
  breaks every consumer's generated headers. Add new fields or messages instead;
  change an existing one only with a deliberate cross-package update.
- **Register new files** in CMakeLists.txt (`add_message_files` /
  `add_service_files` / `add_action_files`) and keep `generate_messages(DEPENDENCIES ...)`
  plus package.xml in sync; add any new dependency package in both places.
  `NavigateTo.action` is registered via `add_action_files` and requires
  `actionlib_msgs` in `find_package`, `generate_messages(DEPENDENCIES ...)`,
  `catkin_package(CATKIN_DEPENDS ...)`, and `package.xml`.
- **Constants live in the .msg** (e.g. LocoStatus MODE_UNKNOWN/DAMP/STAND/WALK =
  0/1/2/3); consumers reference the generated constants, never magic numbers.
- **Canonical authorship/license:** Proprietary, the project owner's GitHub
  noreply identity; this package.xml is the canonical form other packages copy.

## Build / test
```bash
# from catkin_ws/:
catkin build g1_msgs && source devel/setup.bash
catkin run_tests g1_msgs --no-deps && catkin_test_results build/g1_msgs
```
test/test_g1_msgs.cpp is a gtest that constructs the types and checks enum
constants; extend it when you add a message or service.

