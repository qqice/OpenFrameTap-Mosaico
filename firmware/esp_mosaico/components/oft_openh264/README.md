# Decoder dependency

Unmodified Cisco OpenH264 v2.6.0 submodule, commit
`652bdb7719f30b52b08e506645a7322ff1b2cc6f`. Run `git submodule update --init`
after cloning. Preserve vendor/LICENSE when distributing source/binaries.
Only common and decoder C++ sources are linked, no encoder or architecture assembly.
The local sysctl compatibility header answers CPU-count queries only. Missing
pthread scope/scheduling attribute functions reject use with ENOTSUP; decoder
threads are disabled. The firmware owns its low-priority single decoder task.
Two length-overload bridges cover upstream's int32_t declarations versus int
definitions on the S31 newlib ABI (both32-bit, distinct C++ types). No upstream
source or SDK file is patched. Upstream-only compilation uses C++11 and permits
its warnings; the application keeps normal strict warnings.
Local CMake generates copies of selected sources for optional phase-timing
scopes and measured code-placement experiments. Thus the vendor submodule is
unmodified, but compiled copies have explicit local instrumentation. Timers are
disabled outside an explicit on-board replay benchmark. Benchmarks preserve an
immutable compressed sample, hash the RGB output and do not publish replayed
frames as live video. See the preview report for accepted/rejected experiments.
The captured Pocket3 High-profile IDR and following P pictures decode on-board;
this is not blanket compatibility with every High-profile stream. Compilation
uses O3 plus upstream's no-strict-aliasing requirement. Sustained input-rate
decoding remains unachieved; see the preview report. No SDK/camera-firmware change.
