# MBot Autonomy

This repo contains the SLAM and motion controller solution code.

## Installation

To install, we need to build and install binaries, then install the SLAM and motion controller services to run on startup. You can use the install script, from the root of this repo:
```bash
./scripts/install.sh
```
The script will ask for the administrator password.

## template_generate.py

This file is used to generate `mbot-autonomy` repository. 
By running `python3 template_generate.py`, a template codebase will be generated.

To use it, firstly created a new branch from the `main` branch, then run the `template_generate.py` file.

```
// BEGIN task1
void myFunction1() {
    // Some code here...
}
// END task1
```
- Tag the relavent content like this 