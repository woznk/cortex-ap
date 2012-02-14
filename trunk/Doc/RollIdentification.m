z = tf('z', 0.02); rr_tfz=(-0.0001473 * z + 0.000138)/(z^2 - 1.913 * z + 0.9181)
rr_tfs=d2c(rr_tfz)
s = tf('s'); roll_tfs = rr_tfs / s
roll_tfz=c2d(roll_tfs, 0.02)
sisotool(roll_tfz)
