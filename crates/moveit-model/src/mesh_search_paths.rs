// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Resolves a URDF `<mesh filename="package://...">` URI to a file on disk.
//!
//! Not a port: upstream resolves `package://` URIs through
//! `resource_retriever::Retriever`, which walks the real ROS ament package
//! index -- keyed by each package's declared name (from its `package.xml`),
//! not by the name of whatever source directory it happens to live in. This
//! port has no ament index to query, so [`RobotModel`] loading
//! (`RobotModel::constructShape`'s `MESH` case) takes an explicit
//! package-name-to-directory map instead.
//!
//! That map is not a convenience -- it is required, because the two
//! quantities genuinely differ here. Every fixture URDF's mesh URIs name the
//! ROS package `moveit_resources_<robot>_description` (e.g.
//! `package://moveit_resources_panda_description/meshes/collision/link0.stl`),
//! but both real mesh trees this port has access to keep that package's
//! files under a directory named just `<robot>_description` -- the vendored
//! git repository's own source-tree layout, which predates and does not
//! match the ROS package name its own `package.xml` declares. A resolver
//! that joined the URI's package name onto a root directory (assuming the
//! two names match, the way a real ament `share/<package>/` install
//! directory would) would silently resolve nothing against either real
//! layout. An explicit map sidesteps that assumption instead of encoding a
//! `moveit_resources_` prefix-strip as a resolution rule this port has no
//! way to know generalises past these specific packages.
//!
//! Two callers need this, with different maps:
//!
//! - Workspace tests map the packages committed under `fixtures/meshes/`
//!   (see `tools/ci/verify-fixture-provenance.sh`) -- this is what lets
//!   those tests run without the gitignored `third_party/` checkout.
//! - `tools/moveit-diff`'s sweep maps every vendored package directly under
//!   `third_party/moveit_resources/`, not just the ones committed as
//!   fixtures.
//!
//! [`RobotModel`]: crate::RobotModel

use std::collections::HashMap;
use std::path::PathBuf;

/// Package-name-to-directory map for `package://<name>/<relative-path>` mesh
/// URIs. See the module doc for why this is a map keyed by the URI's literal
/// package name, not a search root joined with that name.
#[derive(Debug, Clone, Default)]
pub struct MeshSearchPaths {
    packages: HashMap<String, PathBuf>,
}

impl MeshSearchPaths {
    /// No known packages: every `package://` URI fails to resolve. This is
    /// what a caller not exercising collision-mesh geometry (most of this
    /// port's own tests) passes, reproducing this port's pre-mesh-loading
    /// behaviour of skipping every `<mesh>` collision element with an
    /// [`crate::Diagnostic::UnsupportedLinkGeometry`].
    pub fn none() -> Self {
        Self::default()
    }

    /// Builds the map from `(package_name, directory)` pairs, e.g.
    /// `("moveit_resources_panda_description", ".../panda_description")`.
    /// `directory` is the package's own root -- a resolved path is
    /// `directory.join(relative_path)`, not `directory.join(package_name)`.
    pub fn new<I, K, P>(packages: I) -> Self
    where
        I: IntoIterator<Item = (K, P)>,
        K: Into<String>,
        P: Into<PathBuf>,
    {
        Self {
            packages: packages
                .into_iter()
                .map(|(name, dir)| (name.into(), dir.into()))
                .collect(),
        }
    }

    /// Resolves a `package://<name>/<relative-path>` URI to a file on disk.
    /// Returns `None` for anything else -- a bare filename or another URI
    /// scheme -- since none of this port's fixture URDFs use one (see the
    /// module doc), so there is no real case to fall back on. Also `None`
    /// when `name` names a package not in this map, or when the joined path
    /// is not a file that exists.
    pub fn resolve(&self, resource: &str) -> Option<PathBuf> {
        let rest = resource.strip_prefix("package://")?;
        let (package, relative_path) = rest.split_once('/')?;
        let candidate = self.packages.get(package)?.join(relative_path);
        candidate.is_file().then_some(candidate)
    }

    /// Whether any package was mapped at all -- distinguishes "no search
    /// paths configured" from "search paths configured but this specific
    /// package was not among them" for [`Self::resolve`]'s caller, which
    /// needs the two to produce different diagnostic text.
    pub fn is_empty(&self) -> bool {
        self.packages.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_resolves_nothing() {
        let paths = MeshSearchPaths::none();
        assert!(paths.is_empty());
        assert_eq!(paths.resolve("package://panda_description/link0.stl"), None);
    }

    #[test]
    fn resolves_against_the_mapped_package_directory() {
        let dir = tempfile_dir();
        std::fs::create_dir_all(dir.join("meshes")).unwrap();
        let file = dir.join("meshes/link0.stl");
        std::fs::write(&file, b"stub").unwrap();

        let paths = MeshSearchPaths::new([("moveit_resources_panda_description", dir.clone())]);
        assert_eq!(
            paths.resolve("package://moveit_resources_panda_description/meshes/link0.stl"),
            Some(file)
        );
    }

    #[test]
    fn unknown_package_does_not_resolve() {
        let dir = tempfile_dir();
        let paths = MeshSearchPaths::new([("moveit_resources_panda_description", dir)]);
        assert_eq!(paths.resolve("package://not_a_real_package/x.stl"), None);
    }

    #[test]
    fn non_package_uri_does_not_resolve() {
        let dir = tempfile_dir();
        std::fs::write(dir.join("x.stl"), b"stub").unwrap();

        let paths = MeshSearchPaths::new([("moveit_resources_panda_description", dir)]);
        assert_eq!(paths.resolve("x.stl"), None);
        assert_eq!(paths.resolve("file:///x.stl"), None);
    }

    /// A fresh, unique scratch directory per test, cleaned up on drop by
    /// nobody -- these are tiny and the OS reclaims `/tmp` anyway, matching
    /// this crate's other filesystem-touching tests.
    fn tempfile_dir() -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "moveit-model-mesh-search-paths-test-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }
}
