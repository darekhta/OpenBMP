//! Local scalar Kalman-update algebra smoke tests.
//!
//! These exercise textbook identities on a 1-D scalar Kalman update —
//! the information-form mean / variance composition law — using
//! arbitrary numbers, not specific Bar-Shalom / Li / Kirubarajan
//! examples. Treat them as algebra smoke tests; reproducing
//! published textbook trajectories with their original parameters is
//! deferred work tracked in `docs/roadmap.md`.

#![allow(clippy::float_cmp, clippy::many_single_char_names)]

fn scalar_kalman_update(x: f64, p: f64, z: f64, r: f64) -> (f64, f64, f64) {
    let innovation = z - x;
    let s = p + r;
    let k = p / s;
    let x_post = x + k * innovation;
    let p_post = (1.0 - k) * p * (1.0 - k) + k * r * k;
    let nis = innovation * innovation / s;
    (x_post, p_post, nis)
}

#[test]
fn scalar_update_matches_information_form_identity() {
    // The posterior of two independent Gaussian information sources
    // has variance (P^-1 + R^-1)^-1 and mean weighted by information.
    let prior_x = 10.0;
    let prior_p = 4.0;
    let z = 12.0;
    let r = 9.0;
    let (x_post, p_post, nis) = scalar_kalman_update(prior_x, prior_p, z, r);
    let expected_p = 1.0 / (1.0 / prior_p + 1.0 / r);
    let expected_x = expected_p * (prior_x / prior_p + z / r);
    assert!((p_post - expected_p).abs() < 1.0e-12);
    assert!((x_post - expected_x).abs() < 1.0e-12);
    assert!((nis - (4.0 / 13.0)).abs() < 1.0e-12);
}
