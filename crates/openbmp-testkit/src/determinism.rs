//! Determinism oracle: byte-stable diff utilities.
//!
//! Phase-1 ships the byte-diff primitive. The "run scenario twice and
//! compare telemetry" wrapper lands in Phase-1.8 alongside the first
//! end-to-end golden test, when the kernel and CLI are available.

/// First divergence between two byte streams, including a length
/// mismatch.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ByteDiff {
    /// Byte offset at which the streams first diverge. For length
    /// mismatches this is the length of the shorter stream.
    pub offset: usize,
    /// Expected byte at `offset`, or `None` if the expected stream
    /// is shorter than `offset`.
    pub expected: Option<u8>,
    /// Actual byte at `offset`, or `None` if the actual stream is
    /// shorter than `offset`.
    pub actual: Option<u8>,
}

/// Error returned by [`require_replay_byte_stable`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReplayError<E> {
    /// The first run failed.
    FirstRun {
        /// Underlying runner error.
        source: E,
    },
    /// The second run failed.
    SecondRun {
        /// Underlying runner error.
        source: E,
    },
    /// Both runs succeeded but their bytes differed.
    Diff {
        /// First divergent byte.
        diff: ByteDiff,
    },
}

/// Compare two byte streams for byte-stable equality.
///
/// Returns `Some(ByteDiff)` describing the first divergence, or
/// `None` if the streams are byte-identical.
#[must_use]
pub fn diff_bytes(expected: &[u8], actual: &[u8]) -> Option<ByteDiff> {
    let min_len = expected.len().min(actual.len());
    for i in 0..min_len {
        if expected[i] != actual[i] {
            return Some(ByteDiff {
                offset: i,
                expected: Some(expected[i]),
                actual: Some(actual[i]),
            });
        }
    }
    if expected.len() != actual.len() {
        return Some(ByteDiff {
            offset: min_len,
            expected: expected.get(min_len).copied(),
            actual: actual.get(min_len).copied(),
        });
    }
    None
}

/// Convenience: assert byte-stable equality, returning a structured
/// description on failure.
///
/// # Errors
///
/// Returns the [`ByteDiff`] describing the first divergence.
pub fn require_byte_stable(expected: &[u8], actual: &[u8]) -> Result<(), ByteDiff> {
    match diff_bytes(expected, actual) {
        None => Ok(()),
        Some(diff) => Err(diff),
    }
}

/// Run a deterministic fixture twice and require byte-identical output.
///
/// The closure shape lets Phase 1.8 pass a scenario runner once the
/// kernel exists, while Phase 1.6 can still validate the replay
/// contract with pure byte fixtures.
///
/// # Errors
///
/// Returns [`ReplayError::FirstRun`] or [`ReplayError::SecondRun`] if
/// the runner fails, or [`ReplayError::Diff`] if both runs succeed but
/// produce different bytes.
pub fn require_replay_byte_stable<E>(
    mut run: impl FnMut() -> Result<Vec<u8>, E>,
) -> Result<(), ReplayError<E>> {
    let first = run().map_err(|source| ReplayError::FirstRun { source })?;
    let second = run().map_err(|source| ReplayError::SecondRun { source })?;
    require_byte_stable(&first, &second).map_err(|diff| ReplayError::Diff { diff })
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn identical_streams_have_no_diff() {
        let a = vec![1u8, 2, 3, 4, 5];
        let b = a.clone();
        assert_eq!(diff_bytes(&a, &b), None);
        assert!(require_byte_stable(&a, &b).is_ok());
    }

    #[test]
    fn diff_at_first_byte() {
        let a = vec![0u8, 2, 3];
        let b = vec![1u8, 2, 3];
        let diff = diff_bytes(&a, &b).unwrap();
        assert_eq!(diff.offset, 0);
        assert_eq!(diff.expected, Some(0));
        assert_eq!(diff.actual, Some(1));
    }

    #[test]
    fn diff_in_middle() {
        let a = vec![1u8, 2, 3, 4, 5];
        let b = vec![1u8, 2, 9, 4, 5];
        let diff = diff_bytes(&a, &b).unwrap();
        assert_eq!(diff.offset, 2);
        assert_eq!(diff.expected, Some(3));
        assert_eq!(diff.actual, Some(9));
    }

    #[test]
    fn longer_actual_reports_diff() {
        let a = vec![1u8, 2, 3];
        let b = vec![1u8, 2, 3, 4];
        let diff = diff_bytes(&a, &b).unwrap();
        assert_eq!(diff.offset, 3);
        assert_eq!(diff.expected, None);
        assert_eq!(diff.actual, Some(4));
    }

    #[test]
    fn shorter_actual_reports_diff() {
        let a = vec![1u8, 2, 3, 4];
        let b = vec![1u8, 2, 3];
        let diff = diff_bytes(&a, &b).unwrap();
        assert_eq!(diff.offset, 3);
        assert_eq!(diff.expected, Some(4));
        assert_eq!(diff.actual, None);
    }

    #[test]
    fn empty_streams_match() {
        let a: Vec<u8> = vec![];
        let b: Vec<u8> = vec![];
        assert_eq!(diff_bytes(&a, &b), None);
    }

    #[test]
    fn replay_helper_accepts_identical_runs() {
        let result = require_replay_byte_stable::<()>(|| Ok(vec![1_u8, 2, 3]));
        assert!(result.is_ok());
    }

    #[test]
    fn replay_helper_reports_diff() {
        let mut count = 0_u8;
        let result = require_replay_byte_stable::<()>(|| {
            count += 1;
            Ok(vec![count])
        });
        assert!(matches!(result, Err(ReplayError::Diff { .. })));
    }

    #[test]
    fn replay_helper_reports_second_run_error() {
        let mut count = 0_u8;
        let result = require_replay_byte_stable(|| {
            count += 1;
            if count == 1 {
                Ok(vec![1_u8])
            } else {
                Err("second")
            }
        });
        assert!(matches!(result, Err(ReplayError::SecondRun { .. })));
    }
}
