fn main() {
    let config = slint_build::CompilerConfiguration::new()
        .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer)
        .with_style("fluent".into());
    slint_build::compile_with_config("src/app.slint", config).unwrap();
}
