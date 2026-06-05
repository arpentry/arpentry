//! 2D/3D simplex noise and fractal Brownian motion (port of `gen/noise.c`).
//!
//! Deterministic and continuous, roughly in `[-1, 1]`. The permutation table is
//! the classic Perlin reference table; `perm(k)` indexes it modulo 256, which is
//! identical to the C code's 512-entry table-doubling (`perm[k] == perm[k & 255]`
//! for every `k` the noise functions can form).

/// Perlin reference permutation table (fixed seed).
#[rustfmt::skip]
const PERM_BASE: [u8; 256] = [
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
    36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120,
    234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33,
    88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71,
    134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133,
    230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161,
    1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135, 130,
    116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226, 250,
    124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59, 227,
    47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2, 44,
    154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98,
    108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34,
    242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249, 14,
    239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115, 121,
    50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243,
    141, 128, 195, 78, 66, 215, 61, 156, 180,
];

/// Permutation lookup. The C table is `PERM_BASE` repeated twice; indexing modulo
/// 256 yields the same value for every index the simplex functions produce.
#[inline]
fn perm(k: usize) -> usize {
    PERM_BASE[k & 255] as usize
}

/// 2D gradient vectors.
const GRAD2: [[f64; 2]; 12] = [
    [1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [-1.0, -1.0], [1.0, 0.0], [-1.0, 0.0],
    [0.0, 1.0], [0.0, -1.0], [1.0, 1.0], [-1.0, 1.0], [1.0, -1.0], [-1.0, -1.0],
];

/// 3D gradient vectors (12 cube-edge midpoints).
const GRAD3: [[f64; 3]; 12] = [
    [1.0, 1.0, 0.0], [-1.0, 1.0, 0.0], [1.0, -1.0, 0.0], [-1.0, -1.0, 0.0],
    [1.0, 0.0, 1.0], [-1.0, 0.0, 1.0], [1.0, 0.0, -1.0], [-1.0, 0.0, -1.0],
    [0.0, 1.0, 1.0], [0.0, -1.0, 1.0], [0.0, 1.0, -1.0], [0.0, -1.0, -1.0],
];

#[inline]
fn dot2(g: [f64; 2], x: f64, y: f64) -> f64 {
    g[0] * x + g[1] * y
}

#[inline]
fn dot3(g: [f64; 3], x: f64, y: f64, z: f64) -> f64 {
    g[0] * x + g[1] * y + g[2] * z
}

const F2: f64 = 0.366_025_403_784_438_6; // (sqrt(3) - 1) / 2
const G2: f64 = 0.211_324_865_405_187_13; // (3 - sqrt(3)) / 6

/// 2D simplex noise, roughly in `[-1, 1]`.
pub fn simplex2(x: f64, y: f64) -> f64 {
    // Skew input space to determine the simplex cell.
    let s = (x + y) * F2;
    let i = (x + s).floor();
    let j = (y + s).floor();

    // Unskew back to (x, y) space.
    let t = (i + j) * G2;
    let x0 = x - (i - t);
    let y0 = y - (j - t);

    // Determine which simplex we are in.
    let (i1, j1) = if x0 > y0 { (1.0, 0.0) } else { (0.0, 1.0) };

    let x1 = x0 - i1 + G2;
    let y1 = y0 - j1 + G2;
    let x2 = x0 - 1.0 + 2.0 * G2;
    let y2 = y0 - 1.0 + 2.0 * G2;

    // Hash corner coordinates to gradient indices.
    let ii = (i as i64 & 255) as usize;
    let jj = (j as i64 & 255) as usize;
    let gi0 = perm(ii + perm(jj)) % 12;
    let gi1 = perm(ii + i1 as usize + perm(jj + j1 as usize)) % 12;
    let gi2 = perm(ii + 1 + perm(jj + 1)) % 12;

    // Contributions from the three corners.
    let contrib = |t: f64, gi: usize, dx: f64, dy: f64| -> f64 {
        if t < 0.0 {
            0.0
        } else {
            let t2 = t * t;
            t2 * t2 * dot2(GRAD2[gi], dx, dy)
        }
    };
    let n0 = contrib(0.5 - x0 * x0 - y0 * y0, gi0, x0, y0);
    let n1 = contrib(0.5 - x1 * x1 - y1 * y1, gi1, x1, y1);
    let n2 = contrib(0.5 - x2 * x2 - y2 * y2, gi2, x2, y2);

    70.0 * (n0 + n1 + n2)
}

/// Fractal Brownian motion over [`simplex2`].
pub fn fbm2(x: f64, y: f64, octaves: i32, lacunarity: f64, persistence: f64) -> f64 {
    let mut sum = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut max_amp = 0.0;
    for _ in 0..octaves {
        sum += amplitude * simplex2(x * frequency, y * frequency);
        max_amp += amplitude;
        frequency *= lacunarity;
        amplitude *= persistence;
    }
    sum / max_amp
}

const F3: f64 = 1.0 / 3.0;
const G3: f64 = 1.0 / 6.0;

/// 3D simplex noise, roughly in `[-1, 1]`.
pub fn simplex3(x: f64, y: f64, z: f64) -> f64 {
    // Skew input space to determine the simplex cell.
    let s = (x + y + z) * F3;
    let i = (x + s).floor();
    let j = (y + s).floor();
    let k = (z + s).floor();

    // Unskew back to (x, y, z) space.
    let t = (i + j + k) * G3;
    let x0 = x - (i - t);
    let y0 = y - (j - t);
    let z0 = z - (k - t);

    // Determine which simplex (tetrahedron) we are in: offsets for corners 2/3.
    let (i1, j1, k1, i2, j2, k2) = if x0 >= y0 {
        if y0 >= z0 {
            (1, 0, 0, 1, 1, 0)
        } else if x0 >= z0 {
            (1, 0, 0, 1, 0, 1)
        } else {
            (0, 0, 1, 1, 0, 1)
        }
    } else if y0 < z0 {
        (0, 0, 1, 0, 1, 1)
    } else if x0 < z0 {
        (0, 1, 0, 0, 1, 1)
    } else {
        (0, 1, 0, 1, 1, 0)
    };

    let (fi1, fj1, fk1) = (i1 as f64, j1 as f64, k1 as f64);
    let (fi2, fj2, fk2) = (i2 as f64, j2 as f64, k2 as f64);

    let x1 = x0 - fi1 + G3;
    let y1 = y0 - fj1 + G3;
    let z1 = z0 - fk1 + G3;
    let x2 = x0 - fi2 + 2.0 * G3;
    let y2 = y0 - fj2 + 2.0 * G3;
    let z2 = z0 - fk2 + 2.0 * G3;
    let x3 = x0 - 1.0 + 3.0 * G3;
    let y3 = y0 - 1.0 + 3.0 * G3;
    let z3 = z0 - 1.0 + 3.0 * G3;

    // Hash corner coordinates to gradient indices.
    let ii = (i as i64 & 255) as usize;
    let jj = (j as i64 & 255) as usize;
    let kk = (k as i64 & 255) as usize;
    let gi0 = perm(ii + perm(jj + perm(kk))) % 12;
    let gi1 = perm(ii + i1 + perm(jj + j1 + perm(kk + k1))) % 12;
    let gi2 = perm(ii + i2 + perm(jj + j2 + perm(kk + k2))) % 12;
    let gi3 = perm(ii + 1 + perm(jj + 1 + perm(kk + 1))) % 12;

    let contrib = |t: f64, gi: usize, dx: f64, dy: f64, dz: f64| -> f64 {
        if t < 0.0 {
            0.0
        } else {
            let t2 = t * t;
            t2 * t2 * dot3(GRAD3[gi], dx, dy, dz)
        }
    };
    let n0 = contrib(0.6 - x0 * x0 - y0 * y0 - z0 * z0, gi0, x0, y0, z0);
    let n1 = contrib(0.6 - x1 * x1 - y1 * y1 - z1 * z1, gi1, x1, y1, z1);
    let n2 = contrib(0.6 - x2 * x2 - y2 * y2 - z2 * z2, gi2, x2, y2, z2);
    let n3 = contrib(0.6 - x3 * x3 - y3 * y3 - z3 * z3, gi3, x3, y3, z3);

    32.0 * (n0 + n1 + n2 + n3)
}

/// Fractal Brownian motion over [`simplex3`].
pub fn fbm3(x: f64, y: f64, z: f64, octaves: i32, lacunarity: f64, persistence: f64) -> f64 {
    let mut sum = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0;
    let mut max_amp = 0.0;
    for _ in 0..octaves {
        sum += amplitude * simplex3(x * frequency, y * frequency, z * frequency);
        max_amp += amplitude;
        frequency *= lacunarity;
        amplitude *= persistence;
    }
    sum / max_amp
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simplex_is_deterministic_and_bounded() {
        for k in 0..50 {
            let x = k as f64 * 0.37;
            let y = k as f64 * 0.11;
            let a = simplex2(x, y);
            assert_eq!(a, simplex2(x, y));
            assert!(a >= -1.2 && a <= 1.2, "2d out of range: {a}");
            let b = simplex3(x, y, x - y);
            assert_eq!(b, simplex3(x, y, x - y));
            assert!(b >= -1.2 && b <= 1.2, "3d out of range: {b}");
        }
    }

    #[test]
    fn fbm_averages_octaves() {
        // fBm of a constant-zero field is zero; non-trivial input is bounded.
        let v = fbm3(0.5, -0.3, 0.8, 6, 2.0, 0.5);
        assert!(v.abs() <= 1.0);
    }
}
