## LapUtil: Custom Linux Governor for Laptops

[![DeepSource](https://app.deepsource.com/gh/gg582/laputil.svg/?label=code+coverage&show_trend=true&token=TI2tAytzI2P2dcKbncHMTzfG)](https://app.deepsource.com/gh/gg582/laputil/)
[![DeepSource](https://app.deepsource.com/gh/gg582/laputil.svg/?label=active+issues&show_trend=true&token=TI2tAytzI2P2dcKbncHMTzfG)](https://app.deepsource.com/gh/gg582/laputil/)
[![DeepSource](https://app.deepsource.com/gh/gg582/laputil.svg/?label=resolved+issues&show_trend=true&token=TI2tAytzI2P2dcKbncHMTzfG)](https://app.deepsource.com/gh/gg582/laputil/)

**LapUtil** is a custom CPU frequency governor designed for laptops to extend battery life. It's based on the proven `conservative` governor but optimized with a more aggressive, real-time approach to frequency scaling.

### Key Features
- Based on `conservative`: It utilizes the stable, time-tested logic of the `conservative` governor, which scales CPU frequency incrementally in response to load changes.
- **Optimized for Battery Saving**: LapUtil is specifically tailored for low-power environments, aiming to transition to lower frequencies more quickly when the CPU is under light load.
- **Aggressive, Real-Time Frequency Scaling**: It measures CPU load in real time and aggressively scales down to the minimum frequency when the load is very low. This balances performance by minimizing power consumption without noticeable impact during low-demand tasks.
### Smart Features
#### Adapted Load Smoothing
The governor calculates a smoothed load value using an Exponential Moving Average (EMA) to prevent unnecessary frequency jitter from minor load fluctuations. The smoothing factor (alpha) is not fixed; it is dynamically adjusted based on load volatility.

- High Volatility: When the CPU load changes rapidly, alpha increases, making the governor highly responsive.

- Low Volatility: During steady workloads (like video playback), alpha decreases, prioritizing stability and power efficiency.
- **Dynamic EMA Alpha using Load Delta**: EMA Alpha is automatically calculated via CPU Load delta. 
#### Battery Awareness
Battery Awareness

`laputil` automatically detects whether the system is running on AC power or battery. It then applies a dynamic adjustment to the `powersave_bias` tunable:

- On AC Power: The bias is shifted towards performance.
- On Battery: The bias is shifted towards power saving.

This allows the governor to *seamlessly transition* between performance-focused and efficiency-focused modes without user intervention.
### Install/Update

#### Installation

```bash
su
./scripts/generate_ac_headers.sh
./install.sh
```

#### Update
```bash
su
./scripts/generate_ac_headers.sh
./update.sh
```

#### For Intel Chips(Required)
Add `intel_pstate=passive` to `GRUB_CMDLINE_LINUX_DEFAULT`.
#### For Intel Meteor Lake Chips(Required)
```bash
patch -p1 < lp-e.patch
```

#### Example
`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash intel_pstate=passive"`

### Modprobe Command
```bash
sudo modprobe cpufreq_laputil
```

### Pernament Application
```bash
echo "cpufreq_laputil" | sudo tee /etc/modules-load.d/laputil.conf
```
