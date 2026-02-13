# Setup Methods Comparison Guide

## Overview

There are multiple ways to set up the BLIMP system. This guide explains the differences and helps you choose the right approach for your situation.

### Three Available Methods

1. **INSTALLATION.md** - Official step-by-step manual guide
2. **setup_pi.bash** - Original automated script (reference implementation)
3. **consolidated_setup.sh** - New unified script combining best practices

---

## Method 1: INSTALLATION.md (Manual Setup)

### Description
The official installation guide with detailed explanations for each step. Best if you want to understand what's happening and maintain full control.

### Pros
- ✅ Most detailed explanations
- ✅ Each step can be verified individually
- ✅ Easy to troubleshoot if something fails
- ✅ Can skip optional steps (like GitHub SSH)
- ✅ Educational - learn what each command does

### Cons
- ❌ Takes longer (30-45 minutes)
- ❌ More manual steps to perform
- ❌ Higher chance of typos or mistakes

### When to Use
- **First-time setup** - understand the process
- **Learning purposes** - know what each step does
- **Troubleshooting** - debug specific steps
- **Custom configurations** - modify steps for your needs

### Key Steps (Summary)
1. Boot Ubuntu 22.04 LTS on Raspberry Pi 4B
2. System update and locale configuration
3. Install ROS 2 Humble (ros-humble-desktop)
4. Create workspace and clone repository
5. Enable hardware interfaces (I2C, camera, SPI)
6. Install sensor drivers and tools
7. Build ROS packages
8. Enable services

### Time Estimate
**30-45 minutes** (depending on network speed)

### Setup Command Structure
```bash
# Each step run individually
sudo apt update
sudo apt upgrade -y
# ... more individual commands ...
colcon build
```

---

## Method 2: setup_pi.bash (Original Automated Script)

### Description
The original automated setup script created by the team. Fast, but less documentation about what it does.

### Pros
- ✅ Fast automatic execution (~10-15 minutes)
- ✅ Minimal user input required
- ✅ Used and tested by team
- ✅ Includes GitHub SSH setup

### Cons
- ❌ Less detailed error messages
- ❌ Harder to debug if something fails
- ❌ Uses minimal ROS installation (ros-humble-ros-base)
- ❌ Builds pigpio from source (slower, not recommended)

### When to Use
- **Quick setups** - experienced users only
- **CI/CD automation** - familiar with debugging
- **Team members** - already know this script

### Key Technical Differences
| Component | INSTALLATION.md | setup_pi.bash |
|-----------|-----------------|---------------|
| ROS Package | ros-humble-desktop | ros-humble-ros-base |
| pigpio | apt install (fast) | Source build from GitHub (slow) |
| Locale Setup | ✅ Included | ✅ Included |
| GitHub SSH | ✅ Optional step | ✅ Automatic |
| raspi-config | ✅ Included | ✅ Included |
| Error Handling | Detailed | Minimal |

### Time Estimate
**10-15 minutes** (slower with pigpio source build)

### Setup Command Structure
```bash
# Runs as single script execution
./setup_pi.bash
```

---

## Method 3: consolidated_setup.sh (Recommended - New)

### Description
New unified script that combines the best practices from INSTALLATION.md and setup_pi.bash. Automated but with good error messages and user control.

### Pros (Best of Both Worlds)
- ✅ **Fast** - automated execution (~15-20 minutes)
- ✅ **Smart** - detailed error messages and verification
- ✅ **Flexible** - prompts for optional steps (GitHub SSH, reboot)
- ✅ **Best Practices** - uses ros-humble-desktop (not minimal)
- ✅ **Efficient** - uses apt install for pigpio (not source build)
- ✅ **Well-Organized** - grouped into logical phases
- ✅ **Idempotent** - checks before making changes (won't redo already-done steps)
- ✅ **Educational** - colored output and explanations

### Cons
- ❌ Less granular control than manual INSTALLATION.md
- ❌ Can't skip individual steps (all-or-nothing for each phase)

### When to Use (RECOMMENDED FOR MOST USERS)
- **Most situations** - this is the recommended method
- **First-time setup** - combines speed with reliability
- **Team members** - uses official best practices
- **Automation** - good error handling for scripting

### Key Technical Advantages
```
consolidated_setup.sh includes:
✅ Phase 1: System Configuration + Locale
✅ Phase 2: Optional GitHub SSH setup
✅ Phase 3: ROS 2 Humble (desktop edition)
✅ Phase 4: Workspace creation and clone
✅ Phase 5: Hardware drivers and tools
✅ Phase 6: Package building
✅ Phase 7: Service enablement
✅ Automatic reboot handling
✅ Verification commands
```

### Time Estimate
**15-20 minutes** (fast but with safety checks)

### Setup Command Structure
```bash
# Download and run the script
wget https://raw.githubusercontent.com/.../consolidated_setup.sh
chmod +x consolidated_setup.sh
./consolidated_setup.sh
```

---

## Detailed Comparison Table

| Feature | INSTALLATION.md | setup_pi.bash | consolidated_setup.sh |
|---------|-----------------|---------------|-------------------------|
| **Speed** | ~30-45 min | ~10-15 min | ~15-20 min |
| **User Input Required** | High (many steps) | Low (mostly automatic) | Medium (key choices) |
| **Error Messages** | ✅ Very detailed | ❌ Minimal | ✅ Detailed + colored |
| **Debugging Difficulty** | ✅ Easy | ❌ Hard | ✅ Medium |
| **ROS Edition** | desktop (full) | ros-base (minimal) | desktop (full) ⭐ |
| **pigpio Installation** | apt (fast) ⭐ | source (slow) | apt (fast) ⭐ |
| **Locale Setup** | ✅ Manual steps | ✅ Automatic | ✅ Automatic |
| **GitHub SSH** | ✅ Optional section | ✅ Automatic | ✅ Optional prompt |
| **raspi-config** | ✅ Included | ✅ Included | ✅ Included |
| **Workspace Build** | Manual colcon commands | Automatic | Automatic |
| **I2C Config** | Manual file editing | Automatic | Automatic ⭐ |
| **Service Enablement** | Manual | Automatic | Automatic |
| **Reboot Required** | User must remember | User must remember | Automatic prompt ⭐ |
| **Idempotent** | N/A (manual) | No (may reinstall) | Yes (checks first) ⭐ |
| **Production Ready** | ✅ Yes | ✅ Yes | ✅ Yes (best) |
| **Recommended For** | Learning | Experienced users | Most users ⭐ |

---

## Decision Tree: Which Method Should I Use?

```
Start Here
    ↓
Do you want to UNDERSTAND what you're installing?
├─ YES → Use INSTALLATION.md (manual method)
│         Best for: First-time understanding, troubleshooting
│         Time: 30-45 minutes
│
└─ NO → Do you have Python/bash scripting experience?
    ├─ YES → Use consolidated_setup.sh (automated, best)
    │         Best for: Production systems, most situations
    │         Time: 15-20 minutes
    │
    └─ NO → Also use consolidated_setup.sh!
             It prompts you and gives clear errors
             Time: 15-20 minutes
```

---

## Migration Path: Updating Your System

If you've already set up BLIMP using one method:

### From INSTALLATION.md → consolidated_setup.sh
1. You have the knowledge, now automate future setups
2. Can run consolidated_setup.sh to fill in any gaps
3. The script is idempotent - it won't redo existing setups

### From setup_pi.bash → consolidated_setup.sh
1. backup your current setup (`tar czf blimp_backup.tar.gz ~/blimp_ws`)
2. Run consolidated_setup.sh
3. It will upgrade ROS (ros-base → desktop) if needed
4. Rebuild with `colcon build` after upgrade

### From consolidated_setup.sh → INSTALLATION.md
1. Reference INSTALLATION.md when debugging specific issues
2. Run individual commands from INSTALLATION.md for detailed control
3. Use for understanding specific components

---

## Troubleshooting by Method

### If using INSTALLATION.md
- **I2C not working?** Check Step 7.1 - did you reboot?
- **ROS not found?** Did you run Step 2.7 (source ~/.bashrc)?
- **Build failed?** Check Step 6.1 - build blimp_interfaces first!

### If using setup_pi.bash
- **Stuck on pigpio?** It's building from source - takes 5-10 minutes, be patient
- **Build failed?** Check if you ran as sudo - some steps need elevated privileges
- **Can't find commands?** Did the script reboot automatically?

### If using consolidated_setup.sh
- **Error in Phase X?** The script exits immediately with clear error
- **Want to resume?** Most phases are idempotent - just run again
- **Need to skip GitHub SSH?** The script will ask - select "n"

---

## Network Considerations

All methods require internet access for:
- Ubuntu packages (apt repositories)
- ROS 2 packages and dependencies
- Python packages (pip)
- GitHub repository clone

### Speed Tips
1. **Wired Ethernet** is much faster than Wi-Fi
2. **ROS repositories** can be slow - first download is slower
3. **pigpio source build** (setup_pi.bash only) is very slow - avoid it

**Estimated Download Sizes:**
- Ubuntu packages: ~500 MB
- ROS 2 packages: ~1 GB
- BLIMP repository: ~500 MB
- Total: ~2 GB

---

## Integration with Different Components

### With Docker/Container Setup
- Use consolidated_setup.sh for reproducible builds
- Embed script in Dockerfile for automation

### With CI/CD Pipeline
- setup_pi.bash for fast testing
- consolidated_setup.sh with logging for production

### With Team Collaboration
- Use INSTALLATION.md for documentation
- Use consolidated_setup.sh for automation
- Reference setup_pi.bash for original implementation

---

## Support and Debugging

### Getting Help
1. Check `TROUBLESHOOTING.md` in blimp_src/
2. Review step-by-step in INSTALLATION.md
3. Compare with setup_pi.bash to see original implementation
4. Examine logs from consolidated_setup.sh (check Phase outputs)

### Reporting Issues
When asking for help, mention:
1. Which setup method you used
2. Which step failed (or which phase for scripts)
3. The exact error message
4. Your hardware (Raspberry Pi model, OS version)

---

## Summary

| Use Case | Recommended Method | Time | Learning Curve |
|----------|-------------------|------|-----------------|
| First-time setup | INSTALLATION.md | 30-45 min | Medium (recommended) |
| Production deployment | consolidated_setup.sh | 15-20 min | Low |
| CI/CD automation | consolidated_setup.sh | 15-20 min | Low |
| Understanding the system | INSTALLATION.md | 30-45 min | Learning-focused |
| Quick testing | setup_pi.bash | 10-15 min | High (skip unless experienced) |
| Debug failed setup | INSTALLATION.md | Varies | High (reference) |

### Final Recommendation

**For most users: Use `consolidated_setup.sh`** 

It provides:
- ✅ Best of both worlds (speed + reliability)
- ✅ Clear error messages
- ✅ Intelligent defaults (ros-desktop, apt pigpio)
- ✅ Optional prompts for GitHub SSH
- ✅ Automatic reboot handling
- ✅ Verification commands at the end

Start with consolidated_setup.sh, then reference INSTALLATION.md if you need to understand any steps.
