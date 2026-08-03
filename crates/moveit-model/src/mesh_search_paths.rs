// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Resolves a URDF `<mesh filename="package://...">` URI to a file on disk.
//!
//! Not a port: upstream resolves `package://` URIs through
//! `resource_retriever::Retriever`, which walks the real ROS ament package
//! index. This port has no ROS environment to query, so [`RobotModel`]
//! loading (`RobotModel::constructShape`'s `MESH` case) takes an explicit
//! search-path list instead and applies the one convention every mesh
//! fixture this port has actually needs: a root directory containing one
//! subdirectory per package name.
//!
//! Two callers need this, with different roots:
//!
//! - Workspace tests resolve against `fixtures/meshes/`, which mirrors that
//!   layout for exactly the packages committed there (see
//!   `tools/ci/verify-fixture-provenance.sh`) -- this is what lets those
//!   tests run without the gitignored `third_party/` checkout.
//! - `tools/moveit-diff`'s sweep resolves against
//!   `third_party/moveit_resources/` directly, which has the same
//!   `<package>/<relative-path>` layout for every vendored package, not just
//!   the ones committed as fixtures.
//!
//! A list of roots (rather than one) is what lets a caller combine multiple
//! such directories; nothing here needs more than that, so this deliberately
//! stops at "first root with a matching file wins" rather than modeling
//! ROS's full ament-index package registry (versioned packages, `share/`
//! subdirectories, `AMENT_PREFIX_PATH` search order) -- none of which any
//! caller here has.
//!
//! [`RobotModel`]: crate::RobotModel

use std::path::PathBuf;

/// Search roots for `package://<name>/<relative-path>` mesh URIs. See the
/// module doc for the two real callers and why "one subdirectory per
/// package name, first root wins" is the whole resolution rule.
#[derive(Debug, Clone, Default)]
pub struct MeshSearchPaths {
    roots: Vec<PathBuf>,
}

impl MeshSearchPaths {
    /// No search roots: every `package://` URI fails to resolve. This is
    /// what a caller not exercising collision-mesh geometry (most of this
    /// port's own tests) passes, reproducing this port's pre-mesh-loading
    /// behaviour of skipping every `<mesh>` collision element with an
    /// [`crate::Diagnostic::UnsupportedLinkGeometry`].
    pub fn none() -> Self {
        Self::default()
    }

    /// Roots tried in order for `package://<name>/<relative-path>`, as
    /// `<root>/<name>/<relative-path>`. The first root under which that path
    /// exists as a file wins; a `package://` URI whose package matches no
    /// root under any of them fails to resolve.
    pub fn new<I, P>(roots: I) -> Self
    where
        I: IntoIterator<Item = P>,
        P: Into<PathBuf>,
    {
        Self {
            roots: roots.into_iter().map(Into::into).collect(),
        }
    }

    /// Resolves a `package://<name>/<relative-path>` URI to a file on disk.
    /// Returns `None` for anything else -- a bare filename or another URI
    /// scheme -- since none of this port's fixture URDFs use one (see the
    /// module doc), so there is no real case to fall back on.
    pub fn resolve(&self, resource: &str) -> Option<PathBuf> {
        let rest = resource.strip_prefix("package://")?;
        let (package, relative_path) = rest.split_once('/')?;
        self.roots
            .iter()
            .map(|root| root.join(package).join(relative_path))
            .find(|candidate| candidate.is_file())
    }

    /// Whether any root was given at all -- distinguishes "no search paths
    /// configured" from "search paths configured but this specific package
    /// was not found in any of them" for [`Self::resolve`]'s caller, which
    /// needs the two to produce different diagnostic text.
    pub fn is_empty(&self) -> bool {
        self.roots.is_empty()
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
    fn resolves_against_the_first_root_containing_the_package() {
        let dir = tempfile_dir();
        std::fs::create_dir_all(dir.join("panda_description/meshes")).unwrap();
        let file = dir.join("panda_description/meshes/link0.stl");
        std::fs::write(&file, b"stub").unwrap();

        let paths = MeshSearchPaths::new([dir.clone()]);
        assert_eq!(
            paths.resolve("package://panda_description/meshes/link0.stl"),
            Some(file)
        );
    }

    #[test]
    fn unknown_package_does_not_resolve() {
        let dir = tempfile_dir();
        let paths = MeshSearchPaths::new([dir]);
        assert_eq!(paths.resolve("package://not_a_real_package/x.stl"), None);
    }

    #[test]
    fn non_package_uri_does_not_resolve() {
        let dir = tempfile_dir();
        std::fs::create_dir_all(dir.join("panda_description")).unwrap();
        std::fs::write(dir.join("panda_description/x.stl"), b"stub").unwrap();

        let paths = MeshSearchPaths::new([dir]);
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
