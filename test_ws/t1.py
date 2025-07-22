import numpy as np
import matplotlib.pyplot as plt

# Time array
t = np.linspace(0, 2 * np.pi, 1000)

# Original functions
sin_t = np.sin(t)
cos_t = np.cos(t)
sum_t = sin_t + cos_t

# Equivalent sine wave
amplitude = np.sqrt(2)
phase_shift = np.pi / 4
equivalent_sine = amplitude * np.sin(t + phase_shift)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(t, sin_t, label='sin(t)', linestyle='--')
plt.plot(t, cos_t, label='cos(t)', linestyle='--')
plt.plot(t, sum_t, label='sin(t) + cos(t)', linewidth=2)
plt.plot(t, equivalent_sine, label=f'{amplitude:.2f} * sin(t + π/4)', linestyle=':')
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.title('Demonstration: sin(t) + cos(t) is a Sine Wave')
plt.xlabel('t (radians)')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)
plt.show()
