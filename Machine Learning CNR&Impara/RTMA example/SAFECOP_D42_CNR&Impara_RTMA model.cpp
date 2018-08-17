int ApplyRules(int F0, int m, int d_ms, int d0, int v0) {

  if ((F0 > 1672) && (v0 <= 39)) return 0;
  if ((v0 <= 30)) return 0;
  if ((d0 > 31) && (v0 <= 70)) return 0;
  if ((F0 > 2484) && (d0 > 24 && d0 <= 33) && (v0 <= 88)) return 0;
  if ((m <= 1850) && (d_ms > 10 && d_ms <= 61) && (d0 > 22)) return 0;
  if ((m <= 1707) && (d0 > 18) && (v0 > 36 && v0 <= 50)) return 0;
  if ((F0 > 1610) && (m <= 1149) && (d_ms > 65 && d_ms <= 192) && (d0 > 21) && (v0 > 51 && v0 <= 86)) return 0;
  if ((d_ms > 47 && d_ms <= 48)) return 0;
  if ((F0 <= 4301) && (d0 <= 24) && (v0 > 34)) return 1;
  if ((d0 <= 29) && (v0 > 62)) return 1;
  if ((v0 > 77)) return 1;
  if ((F0 <= 4113) && (m > 1664) && (d_ms > 14) && (v0 > 27)) return 1;
  if ((F0 <= 2547) && (m > 1248 && m <= 1815) && (d_ms > 65)) return 1;
  if ((F0 > 4253) && (v0 > 56)) return 1;

  return 0;
}