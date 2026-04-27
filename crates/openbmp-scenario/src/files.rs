//! Scenario-referenced external files: SHA-256 content addressing.
//!
//! Phase 2.10 introduces external file references for aero decks, motor
//! curves, atmosphere tables, and sensor noise budgets. This module
//! records the scenario-resolved path, computes the SHA-256 digest of
//! the file's bytes once, and (optionally) verifies that digest against
//! a pinned hex string declared in the scenario.
//!
//! The scenario `Scenario::resolved_files()` API exposes the resulting
//! `BTreeMap<field, ResolvedFile>` so the runner can record content
//! hashes in the telemetry header and the determinism gate can fail
//! closed on a pin mismatch.

use std::fs;
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

use crate::error::ScenarioError;

/// Resolved scenario-referenced external file with its content digest.
///
/// The digest is the SHA-256 of the file bytes at load time, encoded as
/// 64 lower-case hexadecimal characters. The struct carries the path
/// after scenario-relative resolution so downstream consumers (telemetry
/// header, snapshot formatter) can render a deterministic representation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ResolvedFile {
    /// Scenario-resolved path to the file.
    pub path: PathBuf,
    /// Lower-case hex SHA-256 digest of the file bytes.
    pub sha256_hex: String,
}

impl ResolvedFile {
    /// Read `path` and return the resolved file with its SHA-256 digest.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::ReferencedFileMissing`] when the file
    /// cannot be read (missing, permission denied, …).
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ScenarioError> {
        let path = path.as_ref();
        let bytes = fs::read(path).map_err(|source| ScenarioError::ReferencedFileMissing {
            path: path.to_path_buf(),
            source,
        })?;
        let mut hasher = Sha256::new();
        hasher.update(&bytes);
        let digest = hasher.finalize();
        let sha256_hex = encode_hex_lower(&digest);
        Ok(Self {
            path: path.to_path_buf(),
            sha256_hex,
        })
    }

    /// Verify that this file's digest matches a declared pin.
    ///
    /// When `expected_hex` is `None` the file is unpinned and any digest
    /// is accepted; otherwise the lower-case hex strings must compare
    /// equal byte-for-byte. The expected pin is normalised to lower
    /// case before comparison so a scenario file that records an
    /// upper-case digest still verifies.
    ///
    /// # Errors
    ///
    /// Returns [`ScenarioError::Sha256Mismatch`] when a pin is provided
    /// and disagrees with the actual digest, or
    /// [`ScenarioError::InvalidSha256Pin`] when the pin is malformed
    /// (not 64 hex characters).
    pub fn verify_pin(&self, expected_hex: Option<&str>) -> Result<(), ScenarioError> {
        let Some(expected) = expected_hex else {
            return Ok(());
        };
        if !is_sha256_hex(expected) {
            return Err(ScenarioError::InvalidSha256Pin {
                path: self.path.clone(),
                value: expected.to_owned(),
            });
        }
        let normalised = expected.to_ascii_lowercase();
        if normalised == self.sha256_hex {
            Ok(())
        } else {
            Err(ScenarioError::Sha256Mismatch {
                path: self.path.clone(),
                expected: normalised,
                actual: self.sha256_hex.clone(),
            })
        }
    }
}

fn encode_hex_lower(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(char::from(HEX[usize::from(byte >> 4)]));
        out.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    out
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.chars().all(|c| c.is_ascii_hexdigit())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    /// Known SHA-256 vector: the empty string maps to
    /// `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`.
    /// Source: NIST CSRC Cryptographic Standards and Guidelines test
    /// vectors, FIPS PUB 180-4 Appendix B.
    const EMPTY_SHA256: &str = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";

    /// Known SHA-256 vector: ASCII string `abc` maps to
    /// `ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad`.
    /// Source: FIPS PUB 180-4 Appendix B test vector 1.
    const ABC_SHA256: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

    fn write_temp(bytes: &[u8]) -> NamedTempFile {
        let mut file = NamedTempFile::new().expect("temp file");
        file.write_all(bytes).expect("write");
        file.flush().expect("flush");
        file
    }

    #[test]
    fn empty_file_matches_known_vector() {
        let file = write_temp(b"");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        assert_eq!(resolved.sha256_hex, EMPTY_SHA256);
    }

    #[test]
    fn abc_matches_known_vector() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        assert_eq!(resolved.sha256_hex, ABC_SHA256);
    }

    #[test]
    fn missing_file_fails_closed() {
        let err = ResolvedFile::load("/nonexistent/openbmp/scenario.toml").unwrap_err();
        assert!(matches!(err, ScenarioError::ReferencedFileMissing { .. }));
    }

    #[test]
    fn correct_pin_verifies() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        resolved.verify_pin(Some(ABC_SHA256)).unwrap();
    }

    #[test]
    fn upper_case_pin_verifies() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        resolved
            .verify_pin(Some(&ABC_SHA256.to_ascii_uppercase()))
            .unwrap();
    }

    #[test]
    fn no_pin_accepts_any_digest() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        resolved.verify_pin(None).unwrap();
    }

    #[test]
    fn mismatched_pin_fails_closed() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        let err = resolved.verify_pin(Some(EMPTY_SHA256)).unwrap_err();
        assert!(matches!(err, ScenarioError::Sha256Mismatch { .. }));
    }

    #[test]
    fn malformed_pin_fails_closed() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        let err = resolved.verify_pin(Some("not-a-valid-pin")).unwrap_err();
        assert!(matches!(err, ScenarioError::InvalidSha256Pin { .. }));
    }

    #[test]
    fn digest_is_stable_across_reloads() {
        let file = write_temp(b"deterministic-bytes");
        let first = ResolvedFile::load(file.path()).unwrap();
        let second = ResolvedFile::load(file.path()).unwrap();
        assert_eq!(first.sha256_hex, second.sha256_hex);
    }

    #[test]
    fn hex_encoding_is_lower_case() {
        let file = write_temp(b"abc");
        let resolved = ResolvedFile::load(file.path()).unwrap();
        assert!(
            resolved
                .sha256_hex
                .chars()
                .all(|c| c.is_ascii_digit() || ('a'..='f').contains(&c)),
        );
    }
}
