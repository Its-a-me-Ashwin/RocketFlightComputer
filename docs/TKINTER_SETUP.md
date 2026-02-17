# Fixing "No module named '_tkinter'" (pyenv on macOS)

The ground station uses **tkinter** for its GUI. If you use **pyenv**, Python is often built without tkinter unless Tcl/Tk is available at install time.

## Option A: Reinstall your pyenv Python with Tcl/Tk (recommended)

### 1. Install Tcl/Tk via Homebrew

```bash
brew install tcl-tk
```

### 2. Reinstall Python with tkinter support

Use your Python version (e.g. 3.12.0). **Apple Silicon (M1/M2):**

```bash
export LDFLAGS="-L$(brew --prefix tcl-tk)/lib"
export CPPFLAGS="-I$(brew --prefix tcl-tk)/include"
export PKG_CONFIG_PATH="$(brew --prefix tcl-tk)/lib/pkgconfig"
export PYTHON_CONFIGURE_OPTS="--with-tcltk-includes='-I$(brew --prefix tcl-tk)/include' --with-tcltk-libs='-L$(brew --prefix tcl-tk)/lib -ltcl8.6 -ltk8.6'"

pyenv uninstall 3.12.0   # then reinstall
pyenv install 3.12.0
```

**Intel Mac** (Homebrew under `/usr/local`): use the same commands; `brew --prefix tcl-tk` will be `/usr/local/opt/tcl-tk`.

### 3. Recreate your virtualenv (if you use one)

```bash
pyenv rehash
# If you use a venv:
cd /path/to/RocketFlightComputer
rm -rf .venv
python -m venv .venv
source .venv/bin/activate
pip install pyserial
python groundStation.py
```

---

## Option B: Use system Python for the ground station only

macOS often ships with a Python that has tkinter. Use it only to run the ground station:

```bash
/usr/bin/python3 groundStation.py
```

Install pyserial for that Python if needed:

```bash
/usr/bin/python3 -m pip install pyserial
```

---

## Option C: Use a different Python build

Install another 3.12 (or 3.11) with a descriptive name, build it with the Tcl/Tk flags above, then use it for this project:

```bash
PYTHON_CONFIGURE_OPTS="--with-tcltk-includes='-I$(brew --prefix tcl-tk)/include' --with-tcltk-libs='-L$(brew --prefix tcl-tk)/lib -ltcl8.6 -ltk8.6'" pyenv install 3.12.1
pyenv local 3.12.1
pip install pyserial
python groundStation.py
```

---

## Verify

```bash
python -c "import tkinter; print('tkinter OK')"
```

If you see `tkinter OK`, run:

```bash
python groundStation.py
```
