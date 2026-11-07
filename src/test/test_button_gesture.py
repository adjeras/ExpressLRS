from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src" / "test" / "test_button_gesture_production.cpp"


def run():
    compiler = next(
        (candidate for candidate in ("g++", "clang++") if _which(candidate)),
        None,
    )
    if compiler is None:
        print("SKIP: no host C++ compiler available")
        return

    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "test_button_gesture_production.exe"
        result = subprocess.run(
            [compiler, "-std=c++17", "-I", str(ROOT / "src" / "lib" / "Handset"), str(SOURCE), "-o", str(output)],
            cwd=ROOT,
            check=False,
        )
        if result.returncode:
            raise SystemExit(result.returncode)
        subprocess.run([str(output)], cwd=ROOT, check=True)


def _which(command):
    result = subprocess.run(
        ["where.exe", command],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


if __name__ == "__main__":
    run()
