z=tf('z',0.02); pr_tfz=(0.0004613 * z - 0.0004608)/(z^2 - 1.958 * z + 0.958)
pr_tfs=d2c(pr_tfz)
s=tf('s');pitch_tfs=pr_tfs/s
pitch_tfz=c2d(pitch_tfs,0.02)
sisotool(pitch_tfz)
