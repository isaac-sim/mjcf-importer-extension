local ext = get_current_extension_info()
project_ext (ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, "isaacsim.asset.importer.mjcf.plugin")
    symbols "On"
    exceptionhandling "On"
    rtti "On"
    staticruntime "On"

    cppdialect "C++17"

    flags { "FatalCompileWarnings", "MultiProcessorCompile", "NoPCH", "NoIncrementalLink" }
    removeflags { "FatalCompileWarnings", "UndefinedIdentifiers" }

    add_files("impl", "plugins")

    includedirs {
        target_deps.."/pybind11/include",
        target_deps.."/nv_usd/%{cfg.buildcfg}/include",
        target_deps.."/usd_ext_physics/%{cfg.buildcfg}/include",
        target_deps.."/python/include",
        target_deps.."/tinyxml2/include",
        target_deps.."/omni_client_library/include",
        target_deps.."/omniverse_asset_converter/include",
    }

    libdirs {
        target_deps.."/nv_usd/%{cfg.buildcfg}/lib",
        target_deps.."/usd_ext_physics/%{cfg.buildcfg}/lib",
        target_deps.."/tinyxml2/lib",
        target_deps.."/omni_client_library/%{cfg.buildcfg}",
        extsbuild_dir.."/omni.usd.core/bin",
        target_deps.."/omniverse_asset_converter/lib",
    }

    links {
        "gf", "tf", "sdf", "vt","usd", "usdGeom", "usdUtils", "usdShade", "usdImaging",
        "usdPhysics", "physicsSchemaTools", "physxSchema", "omni.usd", "tinyxml2", "tbb", "omniclient", "omniverse_asset_converter",
    }

    if os.target() == "linux" then
        includedirs {
            target_deps.."/nv_usd/%{cfg.buildcfg}/include/boost",
            target_deps.."/python/include/python3.10",
        }
        libdirs {

        }
        links {
            "stdc++fs"
        }
    else
        filter { "system:windows", "platforms:x86_64" }
            link_boost_for_windows({"boost_python310"})
        filter {}
        libdirs {
            target_deps.."/tbb/lib/intel64/vc14",
        }
    end

-- Python Bindings for Carobnite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.asset.importer.mjcf.python",
    module = "_mjcf",
    src = "bindings",
    target_subdir = "isaacsim/asset/importer/mjcf"
}

repo_build.prebuild_link {
    { "python/scripts", ext.target_dir.."/isaacsim/asset/importer/mjcf/scripts" },
    { "python/tests", ext.target_dir.."/isaacsim/asset/importer/mjcf/tests" },
    { "docs", ext.target_dir.."/docs" },
    { "data", ext.target_dir.."/data" },
    { "icons", ext.target_dir.."/icons" },
}

if os.target() == "linux" then
    repo_build.prebuild_copy {
        { target_deps.."/tinyxml2/lib/lib**", ext.target_dir.."/bin" },
    }
else
    repo_build.prebuild_copy {
        { target_deps.."/tinyxml2/bin/*.dll", ext.target_dir.."/bin" },
    }
end

repo_build.prebuild_copy {

    { "python/*.py", ext.target_dir.."/isaacsim/asset/importer/mjcf" },
}
