from dataclasses import dataclass, field
from enum import Enum, auto
import os
import shutil
import zipfile
import subprocess
import threading
import glob
from typing import Final

from webserver.runtimemanager import RuntimeManager
from webserver.logger import get_logger, LogParser
from webserver.plugin_config_model import PluginsConfiguration, PluginConfig

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

    def wait_and_finish(proc: subprocess.Popen, step_name: str):
        exit_code = proc.wait()
        build_state.exit_code = exit_code
        if exit_code == 0:
            build_state.status = BuildStatus.SUCCESS
            build_state.log(f"[INFO] {step_name} finished successfully\n")
        else:
            build_state.status = BuildStatus.FAILED
            build_state.log(f"[ERROR] {step_name} failed (exit={exit_code})\n")

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

    # Block until compile finishes
    wait_and_finish(compile_proc, "Build")

    # Stop PLC before cleanup
    runtime_manager.stop_plc()

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

    # Block until cleanup finishes
    wait_and_finish(cleanup_proc, "Cleanup")

    # Restart PLC only if everything succeeded
    if build_state.status == BuildStatus.SUCCESS:
        runtime_manager.reset_crash_tracking()
        runtime_manager.start_plc()
    else:
        build_state.log("[WARNING] PLC program has not been updated because the build failed\n")
