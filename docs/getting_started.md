# Getting Started

## Cloning without Test Data (recommended)

This repository uses Git LFS for large test files (22 MB).
Most users do not need them.

Clone without downloading LFS files:

    GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/TUMFTM/PointCloudCrafter.git

To download them later:

    git lfs pull
