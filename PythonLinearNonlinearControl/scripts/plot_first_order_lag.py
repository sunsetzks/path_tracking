import numpy as np
import matplotlib.pyplot as plt


def first_order_step_response(t: np.ndarray, time_constant: float, gain: float = 1.0) -> np.ndarray:
    """Compute unit-step response of a first-order lag system G(s)=K/(tau s + 1).

    Args:
        t: time vector (seconds)
        time_constant: tau > 0 (seconds)
        gain: static gain K

    Returns:
        y(t) = K * (1 - exp(-t/tau)) for t >= 0
    """
    tau = max(time_constant, 1e-12)
    return gain * (1.0 - np.exp(-t / tau))


def pure_delay_step(t: np.ndarray, delay: float, amplitude: float = 1.0) -> np.ndarray:
    """Compute a delayed unit-step u(t - T).

    Args:
        t: time vector (seconds)
        delay: delay T >= 0 (seconds)
        amplitude: step amplitude

    Returns:
        u(t-T): 0 for t < T, amplitude for t >= T
    """
    return amplitude * (t >= delay).astype(float)


def main() -> None:
    # Time axis
    t_end = 10.0
    dt = 0.001
    t = np.arange(0.0, t_end + dt, dt)

    # Step responses for several time constants
    tau_list = [0.5, 1.0, 2.0]
    colors = ["C0", "C1", "C2"]

    plt.figure(figsize=(8, 5))
    for tau, color in zip(tau_list, colors):
        y = first_order_step_response(t, time_constant=tau, gain=1.0)
        plt.plot(t, y, color=color, label=f"First-order lag, tau={tau}")

    # Pure delay illustration (shape unchanged, only shifted)
    delay_T = 1.5
    u0 = np.ones_like(t)  # ideal unit step at t>=0
    y_delay = pure_delay_step(t, delay=delay_T, amplitude=1.0)
    plt.plot(t, u0, "k--", linewidth=1.2, label="Ideal unit step (no delay)")
    plt.plot(t, y_delay, "k", linewidth=1.2, label=f"Pure delay T={delay_T}s")

    plt.title("Lag vs. Pure Delay: Step Responses")
    plt.xlabel("Time [s]")
    plt.ylabel("Amplitude")
    plt.grid(True, alpha=0.3)
    plt.ylim([-0.05, 1.3])
    plt.xlim([0, t_end])
    plt.legend(loc="right")
    plt.tight_layout()

    # Save to the repository assets folder so it shows up in the workspace
    output_path = "/Users/kuisongzheng/ws/path_tracking/PythonLinearNonlinearControl/assets/firstorderlag.png"
    plt.savefig(output_path, dpi=180)
    print(f"Saved figure to: {output_path}")


if __name__ == "__main__":
    main()


