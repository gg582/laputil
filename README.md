## LapUtil: Custom Linux Governor for Laptops

**LapUtil** is a custom CPU frequency governor designed for laptops to extend battery life. It's based on the proven `conservative` governor but optimized with a more aggressive, real-time approach to frequency scaling.

### Key Features
- Based on `conservative`: It utilizes the stable, time-tested logic of the `conservative` governor, which scales CPU frequency incrementally in response to load changes.
- **Optimized for Battery Saving**: LapUtil is specifically tailored for low-power environments, aiming to transition to lower frequencies more quickly when the CPU is under light load.
- **Aggressive, Real-Time Frequency Scaling**: It measures CPU load in real time and aggressively scales down to the minimum frequency when the load is very low. This balances performance by minimizing power consumption without noticeable impact during low-demand tasks.
### Install/Update

#### Installation

```bash
sudo ./install.sh
```

#### Update
```bash
sudo ./update.sh
```

#### Optional Requirement(For Intel Chips)
Add `intel_pstate=passive` to `GRUB_CMDLINE_LINUX_DEFAULT`.

#### Example
`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash intel_pstate=passive`

### Modprobe Command
```bash
sudo modprobe cpufreq_laputil
```

### Pernament Appliance
Add `cpufreq_laputil` to `/etc/modules`.

