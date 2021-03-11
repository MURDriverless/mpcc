# Model Predictive Contouring Control

### Disclaimer
This repository is a rewrite from the original [MPCC](https://github.com/alexliniger/MPCC) by Alex Liniger (AL-MPCC).
Why the rewrite and not forking ? Our team had several differing software ideas with the original repository, 
so we opted to rewrite by translating the research paper concept into code, relying on AL-MPCC for reference at times. 
Additionally, we only require the C++ version, and we also have a few different constraints from the ones in AL-MPCC.

### Installation
1. Clone this repository: `git clone https://github.com/MURDriverless/mpcc`
2. Install external dependencies: `sudo sh ./install.sh` on Linux, not sure about other systems
3. Compile project: `cmake CMakeLists.txt && make`
4. Run executable: `./mpcc`
