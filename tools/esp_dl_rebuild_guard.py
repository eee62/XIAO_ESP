# Make the detect envs rebuild ESP-IDF when the model component changed.
#
# WHY THIS IS NEEDED
#   custom_sdkconfig makes pioarduino rebuild ESP-IDF from source and install
#   the result into the one framework-arduinoespressif32-libs directory that
#   every env shares. Whether that is needed is decided from a hash of
#   custom_sdkconfig + the MCU + the memory type, kept in the project's
#   sdkconfig.defaults as "# TASMOTA__<hash>". custom_component_add is not in
#   the hash.
#
#   env:wildlife and env:wildlife-face inherit the same custom_sdkconfig from
#   detect_base, so they hash the same. Build one and then the other, which is
#   what `pio run -e wildlife -e wildlife-face` and a fresh machine's first full
#   build both do, and the second is judged up to date: "Compile Arduino IDF
#   libs" never runs, its model component is never fetched, and src/main.cpp
#   dies on
#       fatal error: human_face_detect.hpp: No such file or directory
#   (or pedestrian_detect.hpp, the other way round). The cure by hand was
#   deleting the generated sdkconfig.defaults before every switch.
#
# WHAT THIS DOES
#   Keeps a stamp of the component set the libs were last built for, in
#   sdkconfig.components beside sdkconfig.defaults. When this env's set differs,
#   or there is no stamp, it deletes sdkconfig.defaults, which makes pioarduino
#   treat the libs as stale and rebuild them for this env, fetching its model.
#   Both files are generated and git-ignored (/sdkconfig* in .gitignore).
#
#   The stamp is written here, before the rebuild, not after it. pioarduino
#   starts a nested build once the libs are compiled, and that one runs this
#   script again: it has to find the stamp already matching, or it would delete
#   sdkconfig.defaults again and rebuild forever. If the rebuild then fails,
#   nothing is lost: the failed attempt leaves the libs as they were, and the
#   next run rebuilds because they no longer match what is wanted.
#
#   A switch costs a full ESP-IDF rebuild, ~5 minutes, in either direction.
#   That is inherent in the libs being shared, and is what the manual delete
#   cost as well; this only stops it being forgotten. bench-nodetect is not
#   covered and needs no cover: it has no custom_sdkconfig, so pioarduino
#   reinstalls the stock libs for it by itself.

Import("env")
import hashlib
import os

project_dir = env.subst("$PROJECT_DIR")
defaults = os.path.join(project_dir, "sdkconfig.defaults")
stamp = os.path.join(project_dir, "sdkconfig.components")


def component_set(option):
    """Normalised, order-independent lines of a multi-line project option."""
    lines = (env.GetProjectOption(option, "") or "").splitlines()
    return sorted(line.strip() for line in lines if line.strip())


# Added and removed components both change what the libs contain.
wanted = component_set("custom_component_add") + ["-" + c for c in component_set("custom_component_remove")]
wanted_hash = hashlib.sha256("\n".join(wanted).encode()).hexdigest()[:16]

built_for = None
try:
    with open(stamp) as f:
        built_for = f.read().strip()
except OSError:
    pass

if built_for != wanted_hash:
    if os.path.exists(defaults):
        os.remove(defaults)
        print("esp_dl_rebuild_guard: %s; removed sdkconfig.defaults so ESP-IDF "
              "is rebuilt for %s" % (
                  "libs were built for another component set"
                  if built_for else "no record of which components the libs were built for",
                  ", ".join(c.split("@")[0] for c in wanted) or "this env"))
    with open(stamp, "w") as f:
        f.write(wanted_hash + "\n")
