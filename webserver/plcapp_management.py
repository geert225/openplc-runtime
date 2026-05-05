from dataclasses import dataclass, field
from enum import Enum, auto
import os
import shutil
import time
import zipfile
import subprocess
import threading
import glob
from typing import Final

from webserver.runtimemanager import RuntimeManager
from webserver.logger import get_logger, LogParser
from webserver.plugin_config_model import PluginsConfiguration, PluginConfig, PluginType

logger, _ = get_logger("runtime", use_buffer=True)


MAX_FILE_SIZE: Final[int] = 10 * 1024 * 1024   # 10 MB per file
MAX_TOTAL_SIZE: Final[int] = 50 * 1024 * 1024  # 50 MB total
DISALLOWED_EXT = (".exe", ".dll", ".sh", ".bat", ".js", ".vbs", ".scr")

class BuildStatus(Enum):
    IDLE = auto()
    UNZIPPING = auto()
    COMPILING = auto()
    SUCCESS = auto()
    FAILED = auto()

@dataclass
class BuildProcess:
    status: BuildStatus = BuildStatus.IDLE
    logs: list[str] = field(default_factory=list)
    exit_code: int | None = None

    def log(self, msg: str):
        # logger.info(msg)
        self.logs.append(msg)

    def clear(self):
        self.status = BuildStatus.IDLE
        self.logs.clear()
        self.exit_code = None


build_state = BuildProcess()  # global-ish singleton for status


def analyze_zip(zip_path) -> tuple[bool, list]:
    """Analyze the ZIP file for safety before extraction."""
    build_state.status = BuildStatus.UNZIPPING

    if not zipfile.is_zipfile(zip_path):
        build_state.log("[ERROR] Not a valid PLC Program file.\n")
        return False, []

    with zipfile.ZipFile(zip_path, "r") as zf:
        total_size = 0
        safe = True
        valid_files = []

        for info in zf.infolist():
            filename = info.filename
            uncompressed_size = info.file_size
            compressed_size = info.compress_size
            ext = os.path.splitext(filename)[1].lower()

            # Check for path traversal or absolute paths
            if filename.startswith("/") or ".." in filename or ":" in filename:
                # logger.warning("Dangerous path: %s", filename)
                safe = False

            # Check uncompressed size
            if uncompressed_size > MAX_FILE_SIZE:
                logger.warning("File too large: %s (%d bytes)",
                                filename, uncompressed_size)
                safe = False

            # Check compression ratio (ZIP bomb detection)
            if compressed_size > 0 and uncompressed_size / compressed_size > 1000:
                # logger.warning("Suspicious compression ratio in %s",
                            #    filename)
                safe = False

            # Check disallowed extensions
            if ext in DISALLOWED_EXT:
                logger.warning("Disallowed extension: %s",
                                filename)
                safe = False

            total_size += uncompressed_size
            valid_files.append(info)

        # Check total size
        if total_size > MAX_TOTAL_SIZE:
            # logger.warning("Total uncompressed size too large: %d bytes", 
            #                total_size)
            safe = False

        if safe:
            logger.debug("ZIP file looks safe to extract (based on static checks).")
        else:
            logger.warning("ZIP file failed safety checks.")

        return safe, valid_files


def safe_extract(zip_path, dest_dir, valid_files):
    """Extract files safely to a target directory.
    - Skips macOS metadata (__MACOSX, .DS_Store)
    - Auto-strips a single common root folder if present
    """
    build_state.status = BuildStatus.UNZIPPING

    with zipfile.ZipFile(zip_path, "r") as zf:
        # Detect roots (ignoring macOS junk)
        roots = set()
        for info in valid_files:
            if info.filename.startswith("__MACOSX/") or info.filename.endswith(".DS_Store"):
                continue
            parts = info.filename.split("/", 1)
            if parts and parts[0]:
                roots.add(parts[0])
        strip_root = len(roots) == 1

        for info in valid_files:
            filename = info.filename

            # Normalize path separators for cross-platform compatibility (Windows \ to Unix /)
            filename = filename.replace('\\', '/')

            # Skip macOS junk and directories
            if filename.startswith("__MACOSX/") or filename.endswith(".DS_Store") or filename.endswith("/"):
                continue

            # Optionally strip single root folder
            if strip_root:
                parts = filename.split("/", 1)
                if len(parts) == 2:
                    filename = parts[1]
                else:
                    filename = parts[0]

            out_path = os.path.join(dest_dir, filename)
            out_path = os.path.abspath(out_path)

            # Ensure extraction stays inside destination
            if not out_path.startswith(os.path.abspath(dest_dir)):
                # logger.warning("Skipping suspicious path: %s", filename)
                continue

            os.makedirs(os.path.dirname(out_path), exist_ok=True)

            with zf.open(info) as src, open(out_path, "wb") as dst:
                dst.write(src.read())

            logger.debug("Extracted: %s", out_path)


def update_plugin_configurations(generated_dir: str = "core/generated"):
    """
    Update plugin configurations based on available config files.
    
    Scans generated/conf/ for config files, copies them to plugin directories,
    and updates plugins.conf to enable/disable plugins accordingly.
    """
    plugins_conf_path = "plugins.conf"
    conf_dir = os.path.join(generated_dir, "conf")

    build_state.log(f"[DEBUG] update_plugin_configurations called with generated_dir='{generated_dir}'\n")
    build_state.log(f"[DEBUG] Looking for config files in: {conf_dir}\n")

    # Load current plugin configuration using the dataclass
    plugins_config = PluginsConfiguration.from_file(plugins_conf_path)
    build_state.log(f"[DEBUG] Loaded {len(plugins_config.plugins)} plugins from {plugins_conf_path}\n")
    
    # Log initial state
    for plugin in plugins_config.plugins:
        build_state.log(f"[DEBUG] Initial state - {plugin.name}: enabled={plugin.enabled}, config_path='{plugin.config_path}'\n")

    # Check if conf directory exists
    if not os.path.exists(conf_dir):
        build_state.log(f"[INFO] No conf directory found in {generated_dir}, disabling all plugins\n")
        # When there's no conf directory, disable all currently enabled plugins
        plugins_updated = 0
        update_messages = []
        for plugin in plugins_config.plugins:
            if plugin.enabled:
                plugin.enabled = False
                plugins_updated += 1
                update_messages.append(f"Disabled plugin '{plugin.name}' (no conf directory found)")
        
        # Log the updates
        build_state.log(f"[INFO] Found 0 config files (no conf directory): []\n")
        
        for message in update_messages:
            build_state.log(f"[INFO] {message}\n")
    else:
        # Process config files normally when conf directory exists
        # Use the utility method to update plugins based on available config files
        # Copy config files to plugin directories instead of referencing them directly
        plugins_updated, update_messages = plugins_config.update_plugins_from_config_dir(conf_dir, copy_to_plugin_dirs=True)
        
        # Log the updates
        config_files = glob.glob(os.path.join(conf_dir, "*.json"))
        available_configs = {os.path.splitext(os.path.basename(f))[0]: f for f in config_files}
        build_state.log(f"[INFO] Found {len(available_configs)} config files in {conf_dir}: {list(available_configs.keys())}\n")
        
        for message in update_messages:
            if "Copied config file" in message:
                build_state.log(f"[INFO] {message}\n")
            elif "Enabled plugin" in message or "Disabled plugin" in message:
                build_state.log(f"[INFO] {message}\n")
            else:
                build_state.log(f"[WARNING] {message}\n")

    # Handle VPP plugin registration
    # VPP plugins are compiled from source on-target. If a compiled VPP plugin
    # .so exists in the build directory, register it in plugins.conf.
    build_dir = os.path.join(generated_dir, "build") if generated_dir != "core/generated" else "build"
    vpp_plugin_dir = os.path.join(generated_dir, "vpp_plugin")
    vpp_sos = glob.glob(os.path.join(build_dir, "lib*_plugin.so"))

    if os.path.exists(vpp_plugin_dir) and vpp_sos:
        for so_path in vpp_sos:
            # Extract plugin name: "libsynergy_plugin.so" -> "synergy"
            so_basename = os.path.basename(so_path)
            plugin_name = so_basename.replace("lib", "").replace("_plugin.so", "")

            # Find matching config file
            config_path = os.path.join(conf_dir, f"{plugin_name}.json")
            if not os.path.exists(config_path):
                build_state.log(f"[WARNING] VPP plugin {plugin_name}: no config file found at {config_path}\n")
                continue

            if not plugins_config.has_plugin(plugin_name):
                # First-time deployment: add new entry
                plugins_config.add_plugin(
                    name=plugin_name,
                    path=so_path,
                    enabled=True,
                    plugin_type=PluginType.NATIVE,
                    config_path=config_path,
                )
                plugins_updated += 1
                build_state.log(f"[INFO] Registered new VPP plugin '{plugin_name}' from {so_path}\n")
            else:
                # Update existing entry with new paths and enable
                plugins_config.update_plugin_path(plugin_name, so_path)
                for p in plugins_config.plugins:
                    if p.name == plugin_name:
                        p.config_path = config_path
                        if not p.enabled:
                            p.enabled = True
                            plugins_updated += 1
                        break
                build_state.log(f"[INFO] Updated VPP plugin '{plugin_name}' path to {so_path}\n")
    elif not os.path.exists(vpp_plugin_dir):
        # No VPP plugin in this upload — disable any previously registered VPP plugins
        # that no longer have a matching config file (already handled by
        # update_plugins_from_config_dir above, since their config won't be in conf/)
        pass

    # Save the updated configuration
    if plugins_config.to_file(plugins_conf_path):
        build_state.log(f"[INFO] Plugin configuration update complete. {plugins_updated} plugins updated.\n")
        
        # Log final state
        for plugin in plugins_config.plugins:
            build_state.log(f"[DEBUG] Final state - {plugin.name}: enabled={plugin.enabled}, config_path='{plugin.config_path}'\n")
        
        # Log configuration summary
        summary = plugins_config.get_config_summary()
        build_state.log(f"[INFO] Plugin summary: {summary['enabled']}/{summary['total']} enabled "
                       f"({summary['python']} Python, {summary['native']} Native)\n")
        
        # Validate configurations and log any issues
        issues = plugins_config.validate_plugins()
        if issues:
            build_state.log("[WARNING] Plugin validation issues found:\n")
            for issue in issues:
                build_state.log(f"[WARNING] {issue}\n")
    else:
        build_state.log("[ERROR] Failed to save updated plugin configuration\n")


def _wait_for_plc_idle(runtime_manager: RuntimeManager, timeout_s: float) -> bool:
    """Poll status_plc() until the runtime is NOT in a transition.

    The runtime reports STATUS:TRANSITIONING while a stop/start worker
    is in flight (plc_state has flipped but the unload/load work hasn't
    completed). Any other STATUS (STOPPED, EMPTY, INIT, RUNNING) means
    the runtime is settled and ready to accept the next command.

    Used before the cleanup script runs, so the .so move doesn't race
    against an in-flight unload, and before sending START, so the
    runtime can actually process the command instead of returning
    COMMAND:BUSY. Tolerates the case where the PLC was never started
    (state == INIT or EMPTY) — those return immediately since there's
    no transition to wait for.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        resp = runtime_manager.status_plc()
        if resp and "TRANSITIONING" not in resp.upper():
            return True
        time.sleep(0.1)
    return False


def run_compile(runtime_manager: RuntimeManager, cwd: str = "core/generated", clean: bool = False):
    """Run compile script synchronously (wait for completion) and update status/logs.

    When ``clean=True`` (editor's "Clean build and upload" option), wipe the
    Make-managed ``core/build/`` directory and clear the ccache contents
    before invoking compile.sh. This forces a full recompile from scratch
    even when source content hashes match the cached objects — useful when
    the user suspects a stale or corrupted cache.
    """
    script_path: str = "./scripts/compile.sh"

    build_state.status = BuildStatus.COMPILING
    build_state.log(f"[INFO] Starting compilation\n")

    def stream_output(pipe, prefix):
        for line in iter(pipe.readline, ''):
            msg = f"{prefix}{line}"
            build_state.log(msg)
        pipe.close()

    def wait_step(proc: subprocess.Popen, step_name: str) -> bool:
        """Wait for a subprocess and log the result. Returns True on
        zero-exit. Caller is responsible for combining results — does
        NOT mutate build_state.status, since that's only updated once
        after every step (compile + cleanup) completes, so a successful
        compile isn't masked by a failed cleanup or vice-versa."""
        exit_code = proc.wait()
        if exit_code == 0:
            build_state.log(f"[INFO] {step_name} finished successfully\n")
            return True
        build_state.log(f"[ERROR] {step_name} failed (exit={exit_code})\n")
        return False

    # --- Optional clean step ---
    if clean:
        build_state.log("[INFO] Clean build requested — wiping core/build/ and ccache\n")
        # Wipe the per-project object cache. shutil.rmtree avoids needing
        # `make clean` (which would require the Makefile to be in cwd).
        build_dir = "core/build"
        if os.path.exists(build_dir):
            try:
                shutil.rmtree(build_dir)
            except OSError as e:
                build_state.log(f"[WARNING] Failed to remove {build_dir}: {e}\n")
        # Wipe ccache. Failures here are non-fatal — `ccache -C` returning
        # non-zero (e.g. ccache not installed) shouldn't abort the build,
        # since the build folder wipe alone already invalidates per-file
        # caches that live in build/.
        try:
            ccache_proc = subprocess.run(
                ["ccache", "-C"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            if ccache_proc.returncode == 0:
                build_state.log("[INFO] ccache cleared\n")
            else:
                build_state.log(
                    f"[WARNING] ccache -C exited {ccache_proc.returncode}: "
                    f"{ccache_proc.stderr.strip() or 'no error output'}\n"
                )
        except FileNotFoundError:
            build_state.log("[INFO] ccache not installed — skipping cache wipe\n")

    # --- Compile step ---
    compile_proc = subprocess.Popen(
        ["bash", script_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    threading.Thread(target=stream_output, args=(compile_proc.stdout, ""), daemon=True).start()
    threading.Thread(target=stream_output, args=(compile_proc.stderr, "[ERROR] "), daemon=True).start()

    # Block until compile finishes.
    compile_ok = wait_step(compile_proc, "Build")

    # Stop the running PLC before swapping the .so. stop_plc() returns
    # as soon as the runtime ACKs over the socket, but the actual task
    # / plugin / .so teardown continues asynchronously — wait for the
    # runtime to settle (not in a transition) before letting the
    # cleanup script touch build/new_libplc.so. Otherwise the new .so
    # could be moved into place (or the old one held open) while
    # teardown is still in progress. _wait_for_plc_idle returns
    # immediately for the "PLC was never started" case (state == INIT
    # / EMPTY) — there's no transition to wait for.
    runtime_manager.stop_plc()
    if not _wait_for_plc_idle(runtime_manager, timeout_s=30.0):
        build_state.log(
            "[WARNING] Runtime stayed in TRANSITIONING for 30s; "
            "proceeding with cleanup anyway\n"
        )

    # --- Cleanup step ---
    cleanup_proc = subprocess.Popen(
        ["bash", "./scripts/compile-clean.sh"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1
    )

    threading.Thread(target=stream_output, args=(cleanup_proc.stdout, ""), daemon=True).start()
    threading.Thread(target=stream_output, args=(cleanup_proc.stderr, "[ERROR] "), daemon=True).start()

    cleanup_ok = wait_step(cleanup_proc, "Cleanup")

    # Update build_state.status from the COMBINED result. Previously,
    # only the cleanup result mattered (the second wait_and_finish
    # overwrote whatever the compile set), so a failed compile + a
    # successful cleanup would have been reported as SUCCESS, and a
    # successful compile + a failed cleanup as FAILED — neither
    # matches what actually happened.
    if compile_ok and cleanup_ok:
        build_state.status = BuildStatus.SUCCESS
        build_state.exit_code = 0
    else:
        build_state.status = BuildStatus.FAILED
        build_state.exit_code = 1

    if build_state.status == BuildStatus.SUCCESS:
        # Re-run plugin configuration now that compile.sh has produced any
        # VPP plugin .so files. The pre-compile call at upload time can only
        # register pre-built plugins; VPP plugins are compiled on-target
        # during run_compile, so their entries in plugins.conf have to be
        # written after the compile step succeeds.
        #
        # Hold status back in COMPILING while we finalize plugins.conf so
        # the editor doesn't poll SUCCESS and send START before the VPP
        # plugin entry is written.
        build_state.status = BuildStatus.COMPILING
        update_plugin_configurations(cwd)
        build_state.status = BuildStatus.SUCCESS

        # Reset crash tracking after a successful build — the program changed,
        # so any previous crash pattern no longer applies. Do NOT auto-start
        # the PLC here: the editor is responsible for sending START once it
        # has confirmed a clean build, which gives it control over retries
        # when the previous STOP transition is still finishing (COMMAND:BUSY
        # window).
        runtime_manager.reset_crash_tracking()
    else:
        build_state.log("[WARNING] PLC program has not been updated because the build failed\n")
