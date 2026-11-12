# nlohmann/json (vendored single-header distribution)

- Version: **3.12.0**; pinned, not downloaded during CMake configuration.
- Upstream: <https://github.com/nlohmann/json>
- Release: <https://github.com/nlohmann/json/releases/tag/v3.12.0>
- Release commit: `55f93686c01528224f448c19128836e7df245f72`
- License: MIT; original text is preserved in `LICENSE.MIT`.
- `include/nlohmann/json.hpp` is the unmodified upstream `single_include/nlohmann/json.hpp`.
- Its Git blob SHA-1 was verified against GitHub's contents metadata:
  `82d69f7c5d044c9887c96b90c97f5639083ecd14`.

SHA-256 of the original downloaded files:

```text
aaf127c04cb31c406e5b04a63f1ae89369fccde6d8fa7cdda1ed4f32dfc5de63  include/nlohmann/json.hpp
46a65cffd1ea955132d95a8dd921640714a8d6b537d2e4e482d31145ae95b603  LICENSE.MIT
```

The local `CMakeLists.txt` is a small integration wrapper, not an upstream file.
It declares a header-only target and adds no shared-library dependency.
Consumers manage inclusion in their own CMake files, for example:

```cmake
# Example from this standalone example's protocol/CMakeLists.txt:
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../third_party/nlohmann_json"
                 "${CMAKE_CURRENT_BINARY_DIR}/nlohmann_json")
target_link_libraries(my_target PRIVATE nlohmann_json::nlohmann_json)
```

This is the independent viewer's own pinned copy. Its protocol static library
is shared by the standalone host and the optional board adapter. It does not
depend on another project's headers, SDK targets, or installed JSON package.
No dependency download takes place during CMake configuration.

Verification on 2026-11-05: the pinned header's Git blob ID matched the upstream
metadata. A temporary Windows MinGW GCC 13.2.0 consumer successfully included this
CMake directory twice, compiled the alias target, and ran a JSON round-trip
check. The viewer and optional AArch64 board adapter also compile against this
copy; existing SDK targets are not switched to this dependency.
