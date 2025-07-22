import numpy as np
import matplotlib.pyplot as plt

# 假设采样数据 x 是长度为 N 的数组
N = 1000
t = np.linspace(0, 1, N)
x = np.sin(2 * np.pi * 5 * t) + 0.5 * np.cos(2 * np.pi * 10 * t)  # 5Hz + 10Hz 信号

# 计算 FFT
X = np.fft.fft(x)
freqs = np.fft.fftfreq(N, d=1/N)  # 频率轴

# 绘制频谱
plt.plot(freqs[:N//2], np.abs(X[:N//2]))
plt.xlabel("Frequency (Hz)")
plt.ylabel("Amplitude")
plt.show()