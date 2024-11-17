import os
from SCons.Script import Import

Import("env")

def increment_version(version_file="version.txt"):
    if not os.path.isfile(version_file):
        with open(version_file, "w") as f:
            f.write("1.0.0")

    with open(version_file, "r+") as f:
        version = f.read().strip().split('.')
        version[-1] = str(int(version[-1]) + 1)
        new_version = '.'.join(version)
        f.seek(0)
        f.write(new_version)
        f.truncate()
        print(f"New version (from Python script): {new_version}")  # Debug print
        return new_version

version = increment_version()
env["BUILD_VERSION"] = version

# Pass BUILD_VERSION as a properly quoted string macro
env.Append(CPPDEFINES=[("BUILD_VERSION", f'"{version}"')])

# Debug print for confirmation
print(f"BUILD_VERSION defined as: {env['BUILD_VERSION']}")
