# Security Policy

OpenBMP is an academic simulation-only platform with no operational
deployment claim. The most realistic security concerns are:

- Supply-chain integrity of the published crates and binaries.
- Dependency vulnerabilities (RustSec advisories).
- Build-system or scenario-parser bugs that allow malformed input to
  cause panics, memory unsafety, or unexpected file writes.

Operational-flight, hardware-deployment, or mission-critical-software
security guarantees are explicitly **not in scope**. OpenBMP makes no
compliance claims under IEC 61508, ISO 26262, DO-178C, or equivalent
regimes.

## Reporting a Vulnerability

If you discover a vulnerability:

1. **Do not** open a public GitHub issue.
2. Use GitHub private vulnerability reporting if it is enabled on the
   repository. If it is not enabled yet, contact the repository owner through
   their private profile contact channel and ask for a private disclosure
   channel before sending exploit details.
3. Include: a description, an estimate of impact, reproduction steps,
   and any preliminary remediation ideas.

The first public release is blocked on publishing a maintained security
contact address or enabling GitHub private vulnerability reporting.

## Disclosure Process

- We acknowledge reports within **5 business days**.
- We aim to issue a fix or mitigation within **30 days** of confirmed
  reports for high-severity issues; lower-severity issues may be folded
  into the next regular release.
- Coordinated disclosure: we credit the reporter unless they prefer
  anonymity.
- Security advisories are published as GitHub Security Advisories on
  the repository and (where appropriate) as RustSec advisories.

## Supported Versions

Until the first tagged release, only the `main` branch is supported.
Tagged release support begins with the first 0.1.x release.

## Supply Chain

- The release workflow is configured to produce a CycloneDX SBOM,
  dependency-audit report, build/scenario manifests, and SHA-256 hashes.
  SLSA Build Level 2 provenance remains planned; see
  [`docs/supply-chain.md`](docs/supply-chain.md).
- Yanked dependencies are blocked at CI time via `cargo deny`.
- **Trusted Publishing** remains a planned follow-on; the release workflow
  does not currently publish crates to crates.io.

## Hardening Recommendations for Consumers

OpenBMP is academic software; nevertheless, downstream consumers
embedding OpenBMP into a larger stack should:

- Verify release artifacts against the published hashes and SBOM.
  SLSA provenance verification will apply when attestations are shipped.
- Run `cargo audit` and `cargo deny` against their integration build.
- If adding an async runtime downstream, confine it to host bridge tooling;
  the upstream bridge currently uses blocking transports without one.
- Treat any hardware or deployment expansion as a downstream responsibility
  per [`docs/software-architecture.md`](docs/software-architecture.md)
  § Extensibility for Downstream Integration.
