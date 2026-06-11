//! Floating-point environment guard for deterministic simulation.
//!
//! The default OpenBMP bit-stable profile requires
//! round-to-nearest-ties-to-even with flush-to-zero modes disabled. This module
//! reads the host control register on supported architectures and exposes the
//! decoded state as a small value type so higher layers can fail closed before
//! running deterministic kernels.

use thiserror::Error;

/// Architecture-specific floating-point control register family.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum FpArchitecture {
    /// x86-64 MXCSR.
    X86_64,
    /// AArch64 FPCR.
    Aarch64,
    /// No runtime register reader is compiled for this target.
    Unsupported,
}

/// Decoded floating-point control environment.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FpEnvironment {
    /// Source control-register family.
    pub architecture: FpArchitecture,
    /// Raw control-register bits, when an architecture-specific reader exists.
    pub raw_control: u64,
    /// Flush-to-zero mode is enabled for underflow results.
    pub flush_to_zero: bool,
    /// Denormals-are-zero mode is enabled for denormal inputs.
    pub denormals_are_zero: bool,
    /// AArch64 FPCR.FZ16 is enabled for half-precision values.
    pub half_precision_flush_to_zero: bool,
    /// Rounding-control bits. `0` means round-to-nearest-ties-to-even.
    pub rounding_mode: u32,
}

impl FpEnvironment {
    /// Decode an x86-64 MXCSR register value.
    #[must_use]
    pub const fn from_x86_64_mxcsr(mxcsr: u32) -> Self {
        Self {
            architecture: FpArchitecture::X86_64,
            raw_control: mxcsr as u64,
            flush_to_zero: ((mxcsr >> 15) & 1) != 0,
            denormals_are_zero: ((mxcsr >> 6) & 1) != 0,
            half_precision_flush_to_zero: false,
            rounding_mode: (mxcsr >> 13) & 0b11,
        }
    }

    /// Decode an AArch64 FPCR register value.
    #[must_use]
    pub const fn from_aarch64_fpcr(fpcr: u64) -> Self {
        let flush_to_zero = ((fpcr >> 24) & 1) != 0;
        Self {
            architecture: FpArchitecture::Aarch64,
            raw_control: fpcr,
            flush_to_zero,
            denormals_are_zero: flush_to_zero,
            half_precision_flush_to_zero: ((fpcr >> 19) & 1) != 0,
            rounding_mode: ((fpcr >> 22) & 0b11) as u32,
        }
    }

    /// Return a clean placeholder for targets without a runtime reader.
    #[must_use]
    pub const fn unsupported_clean() -> Self {
        Self {
            architecture: FpArchitecture::Unsupported,
            raw_control: 0,
            flush_to_zero: false,
            denormals_are_zero: false,
            half_precision_flush_to_zero: false,
            rounding_mode: 0,
        }
    }

    /// Read the current thread's floating-point control environment.
    #[must_use]
    pub fn current() -> Self {
        current_fp_environment()
    }

    /// Assert that this environment satisfies the OpenBMP bit-stable profile.
    ///
    /// # Errors
    ///
    /// Returns [`FpEnvironmentDirty`] if any flush-to-zero flag is enabled or
    /// the rounding mode is not round-to-nearest-ties-to-even.
    pub fn assert_clean(self) -> Result<(), FpEnvironmentDirty> {
        if self.flush_to_zero
            || self.denormals_are_zero
            || self.half_precision_flush_to_zero
            || self.rounding_mode != 0
        {
            return Err(FpEnvironmentDirty { observed: self });
        }
        Ok(())
    }

    /// Read and assert the current thread's floating-point environment.
    ///
    /// # Errors
    ///
    /// Returns [`FpEnvironmentDirty`] if [`Self::current`] is not clean.
    pub fn assert_current_clean() -> Result<(), FpEnvironmentDirty> {
        Self::current().assert_clean()
    }
}

/// Floating-point environment did not match the deterministic profile.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Error)]
#[error("floating-point environment is dirty ({observed:?}); cannot guarantee bit-stable replay")]
pub struct FpEnvironmentDirty {
    /// Observed decoded environment.
    pub observed: FpEnvironment,
}

impl FpEnvironmentDirty {
    /// Architecture whose control register was decoded.
    #[must_use]
    pub const fn architecture(&self) -> FpArchitecture {
        self.observed.architecture
    }

    /// Flush-to-zero flag observed in the decoded environment.
    #[must_use]
    pub const fn flush_to_zero(&self) -> bool {
        self.observed.flush_to_zero
    }

    /// Denormals-are-zero flag observed in the decoded environment.
    #[must_use]
    pub const fn denormals_are_zero(&self) -> bool {
        self.observed.denormals_are_zero
    }

    /// AArch64 half-precision flush-to-zero flag.
    #[must_use]
    pub const fn half_precision_flush_to_zero(&self) -> bool {
        self.observed.half_precision_flush_to_zero
    }

    /// Rounding-control bits.
    #[must_use]
    pub const fn rounding_mode(&self) -> u32 {
        self.observed.rounding_mode
    }
}

#[cfg(target_arch = "x86_64")]
fn current_fp_environment() -> FpEnvironment {
    FpEnvironment::from_x86_64_mxcsr(read_x86_64_mxcsr())
}

#[cfg(target_arch = "aarch64")]
fn current_fp_environment() -> FpEnvironment {
    FpEnvironment::from_aarch64_fpcr(read_aarch64_fpcr())
}

#[cfg(not(any(target_arch = "x86_64", target_arch = "aarch64")))]
fn current_fp_environment() -> FpEnvironment {
    FpEnvironment::unsupported_clean()
}

#[cfg(target_arch = "x86_64")]
#[allow(unsafe_code)]
fn read_x86_64_mxcsr() -> u32 {
    let mut csr = 0_u32;
    let csr_ptr = core::ptr::addr_of_mut!(csr);
    // SAFETY: `stmxcsr` stores the MXCSR control register into the provided
    // 32-bit memory location. `csr_ptr` points to a live local `u32`, is
    // properly aligned, and is valid for this single write.
    unsafe {
        core::arch::asm!(
            "stmxcsr [{0}]",
            in(reg) csr_ptr,
            options(nostack, preserves_flags),
        );
    }
    csr
}

#[cfg(target_arch = "aarch64")]
#[allow(unsafe_code)]
fn read_aarch64_fpcr() -> u64 {
    let fpcr: u64;
    // SAFETY: `mrs <Xt>, fpcr` copies the current thread's FPCR into a general
    // register. It does not read memory, write memory, alter the stack, or
    // modify condition flags.
    unsafe {
        core::arch::asm!(
            "mrs {0}, fpcr",
            out(reg) fpcr,
            options(nomem, nostack, preserves_flags),
        );
    }
    fpcr
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn clean_decoded_environments_are_accepted() {
        FpEnvironment::from_x86_64_mxcsr(0).assert_clean().unwrap();
        FpEnvironment::from_aarch64_fpcr(0).assert_clean().unwrap();
        FpEnvironment::unsupported_clean().assert_clean().unwrap();
    }

    #[test]
    fn x86_64_mxcsr_decoder_rejects_dirty_modes() {
        let ftz = FpEnvironment::from_x86_64_mxcsr(1 << 15)
            .assert_clean()
            .unwrap_err();
        assert!(ftz.flush_to_zero());
        assert!(!ftz.denormals_are_zero());
        assert_eq!(ftz.rounding_mode(), 0);

        let daz = FpEnvironment::from_x86_64_mxcsr(1 << 6)
            .assert_clean()
            .unwrap_err();
        assert!(!daz.flush_to_zero());
        assert!(daz.denormals_are_zero());

        let round_down = FpEnvironment::from_x86_64_mxcsr(1 << 13)
            .assert_clean()
            .unwrap_err();
        assert_eq!(round_down.rounding_mode(), 1);
    }

    #[test]
    fn aarch64_fpcr_decoder_rejects_dirty_modes() {
        let fz = FpEnvironment::from_aarch64_fpcr(1 << 24)
            .assert_clean()
            .unwrap_err();
        assert_eq!(fz.architecture(), FpArchitecture::Aarch64);
        assert!(fz.flush_to_zero());
        assert!(fz.denormals_are_zero());

        let fz16 = FpEnvironment::from_aarch64_fpcr(1 << 19)
            .assert_clean()
            .unwrap_err();
        assert!(!fz16.flush_to_zero());
        assert!(fz16.half_precision_flush_to_zero());

        let round_down = FpEnvironment::from_aarch64_fpcr(1 << 22)
            .assert_clean()
            .unwrap_err();
        assert_eq!(round_down.rounding_mode(), 1);
    }

    #[test]
    fn current_environment_is_clean_under_default_test_runner() {
        FpEnvironment::assert_current_clean().expect("default test FP environment must be clean");
    }
}
