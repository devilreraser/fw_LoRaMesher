import os


# def set_linker_priority(target, source, env):
#     print("Running set_linker_priority...")
#     build_dir = env.subst("$BUILD_DIR")
#     stm32wlxx_hal_msp_path = os.path.join(build_dir, "src/stm32wlxx_hal_msp.c.o")

#     if os.path.isfile(stm32wlxx_hal_msp_path):
#         print(f"Prioritizing: {stm32wlxx_hal_msp_path}")
#         # Add the object file to the start of LINKFLAGS
#         env.Append(LINKFLAGS=[stm32wlxx_hal_msp_path])
#     else:
#         print(f"File not found: {stm32wlxx_hal_msp_path}")

# try:
#     Import("env")
#     # Add the action to the build process
#     #env.AddPreAction("checkprogsize", set_linker_priority)
#     #env.AddPreAction("buildprog", set_linker_priority)
#     env.AddPreAction("link", set_linker_priority)
# except NameError:
#     print("This script is for PlatformIO and cannot be run standalone.")




import os

print("Executing linker_priority.py script")
def set_linker_priority(target, source, env):
    """
    Ensure stm32wlxx_hal_msp.c.o is prioritized during the linking process.
    """
    print("Running set_linker_priority...")
    build_dir = env.subst("$BUILD_DIR")
    stm32wlxx_hal_msp_path = os.path.join(build_dir, "src/stm32wlxx_hal_msp.c.o")
    analog_path = os.path.join(build_dir, "src/analog.cpp.o")

    if os.path.isfile(stm32wlxx_hal_msp_path):
        print(f"Prioritizing: {stm32wlxx_hal_msp_path}")
        # Ensure stm32wlxx_hal_msp.c.o is added at the start of linker flags
        env.Prepend(LINKFLAGS=[stm32wlxx_hal_msp_path])
    else:
        print(f"File not found: {stm32wlxx_hal_msp_path}")

    if os.path.isfile(analog_path):
        print(f"Deprioritizing: {analog_path}")
        # Deprioritize analog.cpp.o by adding it after other flags
        env.Append(LINKFLAGS=[analog_path])
    else:
        print(f"File not found: {analog_path}")

try:
    Import("env")
    # Add the custom action to the build process
    #env.AddPreAction("buildprog", set_linker_priority)
    env.Execute(set_linker_priority)
except NameError:
    print("This script is for PlatformIO and cannot be run standalone.")

