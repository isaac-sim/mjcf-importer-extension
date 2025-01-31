local ext = get_current_extension_info()
ext.target_dir = "%{root}/_build/%{platform}/%{config}/extsDeprecated/"..ext.id

project_ext (ext)

repo_build.prebuild_link {
    { "docs", ext.target_dir.."/docs" },
    { "omni", ext.target_dir.."/omni"},
    { "data", ext.target_dir.."/data"},
}
