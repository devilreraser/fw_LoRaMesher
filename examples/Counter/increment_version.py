import os

def increment_version(version_file="version.txt"):
    print("Running increment_version...")
    
    # Ensure the version file exists
    if not os.path.isfile(version_file):
        with open(version_file, "w") as f:
            f.write("1.0.0")

    # Increment version
    with open(version_file, "r+") as f:
        version = f.read().strip().split('.')
        version[-1] = str(int(version[-1]) + 1)
        new_version = '.'.join(version)
        f.seek(0)
        f.write(new_version)
        f.truncate()
        print(f"New version: {new_version}")
        return new_version

# PlatformIO-specific logic
try:
    Import("env")

    def before_build_action(target, source, env):
        version = increment_version()
        env.Append(CPPDEFINES=[("BUILD_VERSION", '\\"' + version + '\\"')])

    # Ensure the action happens before any build process
    env.AddPreAction("checkprogsize", before_build_action)
    env.AddPreAction("buildprog", before_build_action)

except NameError:
    # Standalone execution fallback
    if __name__ == "__main__":
        increment_version()
