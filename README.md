# LapUtil: An Intelligent, Power-Aware CPU Governor for Laptops

[![DeepSource](https://app.deepsource.com/gh/gg582/laputil.svg/?label=code+coverage&show_trend=true&token=TI2tAytzI2P2dcKbncHMTzfG)](https://app.deepsource.com/gh/gg582/laputil/)  
[![DeepSource](https://app.deepsource.com/gh/gg582/laputil.svg/?label=active+issues&show_trend=true&token=TI2tAytzI2P2dcKbncHMTzfG)](https://app.deepsource.com/gh/gg582/laputil/)  
[![DeepSource](https://app.deepsource.com/gh/gg582/laputil.svg/?label=resolved+issues&show_trend=true&token=TI2tAytzI2P2dcKbncHMTzfG)](https://app.deepsource.com/gh/gg582/laputil/)  

**LapUtil** is a custom CPU frequency governor built for laptops, designed to balance **performance and battery life** more intelligently than traditional load-based governors.  
Instead of reacting only to raw CPU utilization, LapUtil employs a **predictive, Adam-inspired optimizer** combined with **dynamic load smoothing** and **hybrid safety heuristics**.  

---

## How It Works: Core Concepts

### 1. Adam-Inspired Predictive Optimizer
LapUtil adapts the ideas of the *Adam* optimizer from machine learning to manage CPU frequency using **power consumption trends**, not just CPU load.  

- **m (Momentum):** Tracks the *average directional trend* of power consumption: rising, stable, or falling.  
- **v (Volatility):** Measures how *unstable or spiky* the power usage is.  

By combining these factors in an `m / sqrt(v)`-like manner, LapUtil responds only to **sustained changes** in power usage, avoiding jitter from short spikes while anticipating future demand.  

### 2. Dynamic Load Smoothing (EMA)
Minor load fluctuations shouldn't cause frequent frequency jumps. LapUtil applies an **Exponential Moving Average (EMA)** to CPU load, but with a twist:  

- **High load volatility:** The smoothing factor (alpha) increases, making the governor more responsive.  
- **Stable load:** Alpha decreases, keeping frequencies steady and efficient.  

This adaptive EMA prevents jitter while remaining agile when needed.  

### 3. Hybrid Heuristics & Overrides
To ensure responsiveness in edge cases, LapUtil blends predictive optimization with traditional load-based rules:  

- **High Load Override:** If load exceeds `up_threshold`, LapUtil immediately boosts frequency for responsiveness.  
- **Low Load Override:** If load drops below `down_threshold`, frequency is reduced quickly for maximum efficiency.  

### 4. Battery & Cluster Awareness
- **On Battery:** Predictive optimization is enabled to extend runtime.  
- **On AC Power:** The optimizer is bypassed, favoring raw performance.  
- **P-cores / E-cores:** When battery is low, LapUtil prioritizes efficiency cores (E-cores) to reduce power draw further.  

---

## Installation and Usage

### Prerequisites
Install kernel headers and build tools first (Debian/Ubuntu example):  
```bash
sudo apt update
sudo apt install linux-headers-$(uname -r) build-essential
```

### Installation
```bash
su
./scripts/generate_ac_headers.sh
./install.sh
```

### Update
```bash
su
./scripts/generate_ac_headers.sh
./update.sh
```

### Intel CPU Configuration
If the `intel_pstate` driver is active, `acpi-cpufreq` governors cannot run. Disable it:  

1. Edit `/etc/default/grub`.  
2. Add `intel_pstate=passive` to `GRUB_CMDLINE_LINUX_DEFAULT`.  
   Example:  
   ```
   GRUB_CMDLINE_LINUX_DEFAULT="quiet splash intel_pstate=passive"
   ```
3. Update and reboot:  
   ```bash
   sudo update-grub
   sudo reboot
   ```

#### For Intel Meteor Lake Chips
```bash
patch -p1 < lp-e.patch
```

---

## Activating the Governor

### Temporary Activation (until reboot)
```bash
sudo modprobe cpufreq_laputil
sudo cpupower frequency-set -g laputil
```

### Permanent Activation (at boot)
```bash
echo "cpufreq_laputil" | sudo tee /etc/modules-load.d/laputil.conf
```
(Optionally set LapUtil as the default governor using `cpupower-gui`, TLP, or similar tools.)

---

## Tunables (via Sysfs)

Available under:  
`/sys/devices/system/cpu/cpufreq/policy*/laputil/`  

| Parameter              | Description                                                          | Default |
| ---------------------- | -------------------------------------------------------------------- | ------- |
| `up_threshold`         | CPU load (%) above which frequency is raised                        | 75      |
| `down_threshold`       | CPU load (%) below which frequency is lowered                       | 10      |
| `freq_step`            | Step size for frequency change, as % of max frequency               | 5       |
| `sampling_rate`        | Sampling period (seconds)                                           | 1       |
| `sampling_down_factor` | Multiplier applied to sampling rate when scaling frequency down     | 2       |
| `ignore_nice_load`     | Ignore `nice` processes in load calculation (1 = ignore)            | 1       |

---

## License
This project is licensed under the **GNU General Public License v2.0**.  