# Name the esp-dl component archives on the final link line.
#
# WHY THIS IS NEEDED
#   custom_sdkconfig makes pioarduino rebuild ESP-IDF from source ("*** Compile
#   Arduino IDF libs ***"). That step compiles every managed component --
#   including the ones pulled in by custom_component_add -- archives each one,
#   installs the archives into the framework-arduinoespressif32-libs package,
#   and then deletes its own build tree.
#
#   The final Arduino link draws its libraries from that installed set. The
#   stock components are named there explicitly; components added through
#   custom_component_add are not. So esp-dl and the detection model compile
#   cleanly, archive cleanly, install cleanly -- and are then simply never
#   named on the link line. That is why PedestrianDetect::PedestrianDetect,
#   dl::detect::DetectWrapper::set_score_thr and dl::image::sw_decode_jpeg come
#   back as undefined references from a build where nothing appears to be wrong.
#
#   Confirmed by reading the failing link's own map file: it LOADs
#   libespressif__dl_fft.a -- a stock component -- out of the framework lib
#   directory, and never mentions libespressif__esp-dl.a sitting right beside
#   it. nm on those archives shows all three missing symbols defined (T).
#
# WHAT THIS DOES
#   Adds -l for the missing archives. No -L is needed: the framework lib
#   directory is already on the link line, which is how the stock components
#   resolve.
#
#   Order matters and grouping is not available. SCons builds this list with
#   -l prefixed to every plain string, so a literal "-Wl,--start-group" here
#   becomes "-l-Wl,--start-group" and the link dies looking for a library by
#   that name. The names are therefore appended in dependency order instead --
#   each model archive first, then esp-dl, which the models require -- so every
#   reference resolves forwards, the only direction a linker searches.
#
#   Only archives that actually exist are named, so an env whose component set
#   changed does not fail the link with a bogus -l before the real error.

Import("env")

import os

platform = env.PioPlatform()
mcu = env.BoardConfig().get("build.mcu", "esp32s3")

libs_pkg = platform.get_package_dir("framework-arduinoespressif32-libs")
lib_dir = os.path.join(libs_pkg, mcu, "lib") if libs_pkg else None


def component_to_lib(entry):
    """'espressif/pedestrian_detect@0.3.2' -> 'espressif__pedestrian_detect'"""
    return entry.split("@")[0].strip().replace("/", "__")


# Models first, then esp-dl: they require it, so it has to come after them.
wanted = []
for line in (env.GetProjectOption("custom_component_add", "") or "").splitlines():
    line = line.strip()
    if line:
        wanted.append(component_to_lib(line))

# esp-dl is a transitive dependency of the models rather than something
# custom_component_add names, so it has to be listed explicitly.
wanted.append("espressif__esp-dl")

# esp-dl's own back end is two genuinely precompiled archives that ship inside
# the component sources and are never installed into the framework lib dir, so
# unlike the above these need a -L as well. They come last: esp-dl calls into
# them, not the other way round.
#   fbs_model     -- the flatbuffer model loader (fbs::FbsModel)
#   esp_new_jpeg  -- the JPEG decoder behind dl::image::sw_decode_jpeg
PREBUILT = (
    ("espressif__esp-dl/fbs_loader/lib", "fbs_model"),
    ("espressif__esp_new_jpeg/lib", "esp_new_jpeg"),
)

managed = os.path.join(env.subst("$PROJECT_DIR"), "managed_components")

# Where these archives live depends on when the link happens:
#
#   incremental build -- the framework lib dir, already on the link line,
#                        installed there by an earlier IDF-libs step;
#   clean build       -- $BUILD_DIR/esp-idf/<component>/, because the IDF-libs
#                        step links before it installs anything.
#
# Naming a -L for the build-tree location as well covers both, and costs
# nothing when the directory does not exist.
build_dir = env.subst("$BUILD_DIR")
for name in wanted:
    env.Append(LIBPATH=[os.path.join(build_dir, "esp-idf", name)])

libs = env.get("LIBS", [])
existing = {str(x) for x in libs}

missing_on_disk = []
to_add = []
for name in wanted:
    if name in existing:
        continue            # already named by the platform; leave it alone
    # Deliberately NOT gated on the archive existing yet. On a clean build the
    # "Compile Arduino IDF libs" step has only been *scheduled* by the time
    # this script runs -- it installs these archives later, during the build
    # phase -- so an existence check here reports them missing and silently
    # drops them, which is exactly the bug this script exists to fix. If one
    # really is absent the linker says "cannot find -lX", which is clear
    # enough.
    to_add.append(name)

for subdir, libname in PREBUILT:
    d = os.path.join(managed, subdir, mcu)
    if libname in existing:
        continue
    if not os.path.isfile(os.path.join(d, "lib%s.a" % libname)):
        missing_on_disk.append(libname)
        continue
    env.Append(LIBPATH=[d])
    to_add.append(libname)

if missing_on_disk:
    print("esp_dl_link: not naming (no archive found): %s"
          % ", ".join(missing_on_disk))

if to_add:
    env.Append(LIBS=to_add)
    print("esp_dl_link: linking %s" % ", ".join(to_add))
    if lib_dir:
        absent = [n for n in to_add
                  if not os.path.isfile(os.path.join(lib_dir, "lib%s.a" % n))
                  and not any(os.path.isfile(os.path.join(str(d), "lib%s.a" % n))
                              for d in env.get("LIBPATH", []))]
        if absent:
            print("esp_dl_link: (not installed yet, expected on a clean "
                  "build: %s)" % ", ".join(absent))
