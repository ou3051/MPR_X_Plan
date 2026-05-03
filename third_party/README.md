# Third-Party Dependencies

This directory is reserved for explicitly fetched third-party source trees.

Dependencies must not be downloaded implicitly during CMake configure. Use a
documented command or script, pin the dependency to a tag/commit, and record it
in the SOUP list before using it for regulated builds.

Initial allowed fallback dependencies:

| Dependency | Default source | Pin | Purpose |
| --- | --- | --- | --- |
| GoogleTest | https://github.com/google/googletest.git | v1.14.0 / f8d7d77c06936315286eb55f8de22cd23c188571 | Unit and validation tests |
| DCMTK | https://github.com/DCMTK/dcmtk.git | DCMTK-3.6.8 / 139972c69896afdbcc5e58828e017b3b9c26cbf3 | DICOM parsing fallback |
