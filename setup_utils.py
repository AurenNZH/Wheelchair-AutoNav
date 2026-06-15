#!/usr/bin/env python3
"""
Setup utility for Wheelchair Teleoperation Package
Helps with system setup and verification
"""

import subprocess
import sys
import os
import shutil


def run_command(cmd, description="", check=True):
    """Run a shell command and return success status."""
    print(f"\n[*] {description}")
    print(f"    $ {cmd}")
    
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode != 0:
            if check:
                print(f"    ERROR: Command failed")
                if result.stderr:
                    print(f"    {result.stderr}")
                return False
            else:
                print(f"    Warning: Command returned non-zero status")
        
        if result.stdout:
            for line in result.stdout.strip().split('\n'):
                print(f"    {line}")
        
        return result.returncode == 0
    
    except Exception as e:
        print(f"    Exception: {e}")
        return False


def check_can_utils():
    """Check if can-utils is installed."""
    print("\n" + "="*60)
    print("CHECKING CAN-UTILS INSTALLATION")
    print("="*60)
    
    if shutil.which("candump") and shutil.which("cansend"):
        print("[✓] can-utils is installed")
        return True
    else:
        print("[✗] can-utils NOT found")
        print("\nTo install on Raspberry Pi OS:")
        print("  sudo apt-get update")
        print("  sudo apt-get install can-utils")
        return False


def check_python_packages():
    """Check Python dependencies."""
    print("\n" + "="*60)
    print("CHECKING PYTHON DEPENDENCIES")
    print("="*60)
    
    required_packages = {
        "yaml": "pyyaml",
    }
    
    all_ok = True
    
    for module, package in required_packages.items():
        try:
            __import__(module)
            print(f"[✓] {package} is installed")
        except ImportError:
            print(f"[✗] {package} NOT installed")
            all_ok = False
    
    return all_ok


def check_can_interface():
    """Check if CAN interface is available."""
    print("\n" + "="*60)
    print("CHECKING CAN INTERFACE")
    print("="*60)
    
    result = run_command(
        "ip link show | grep can",
        "Looking for CAN interfaces...",
        check=False
    )
    
    if result:
        # Check if any interface is UP
        result = run_command(
            "ip link show can0",
            "Checking can0 status...",
            check=False
        )
        
        if result:
            return True
    
    print("\n[!] No active CAN interface found")
    print("\nTo set up CAN interface on Raspberry Pi:")
    print("  sudo ip link set can0 up type can bitrate 125000")
    print("\nFor permanent configuration, add to /boot/config.txt:")
    print("  dtparam=spi=on")
    print("  dtoverlay=mcp2515-can0-overlay,oscillator=16000000,interrupt=25")
    print("  dtoverlay=spi-bcm2835-overlay")
    print("\nThen reboot:")
    print("  sudo reboot")
    
    return False


def check_wheelchair_power():
    """Check if wheelchair is powered on (by monitoring CAN bus)."""
    print("\n" + "="*60)
    print("CHECKING WHEELCHAIR CONNECTION")
    print("="*60)
    
    print("[*] Listening for CAN frames (5 seconds timeout)...")
    
    result = subprocess.run(
        "timeout 5 candump can0 2>/dev/null | head -1",
        shell=True,
        capture_output=True,
        text=True
    )
    
    if result.stdout.strip():
        print(f"[✓] Received CAN frame:")
        print(f"    {result.stdout.strip()}")
        print("\n[✓] Wheelchair appears to be powered on and connected")
        return True
    else:
        print("[✗] No CAN frames detected")
        print("\nPlease check:")
        print("  1. Is wheelchair powered on?")
        print("  2. Is CAN bus properly connected?")
        print("  3. Is can0 interface up? (ip link show can0)")
        return False


def install_dependencies():
    """Install Python dependencies."""
    print("\n" + "="*60)
    print("INSTALLING PYTHON DEPENDENCIES")
    print("="*60)
    
    # Get directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    requirements_file = os.path.join(script_dir, "requirements.txt")
    
    if not os.path.exists(requirements_file):
        print("[-] requirements.txt not found")
        return False
    
    run_command(
        f"pip install -r {requirements_file}",
        "Installing dependencies from requirements.txt..."
    )
    
    return check_python_packages()


def main():
    """Run all checks."""
    print("\n╔════════════════════════════════════════════════════════════╗")
    print("║   Wheelchair Teleoperation - System Setup Verification    ║")
    print("╚════════════════════════════════════════════════════════════╝")
    
    results = {
        "CAN-Utils": check_can_utils(),
        "Python Deps": check_python_packages(),
        "CAN Interface": check_can_interface(),
        "Wheelchair": check_wheelchair_power(),
    }
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    for check, passed in results.items():
        status = "[✓]" if passed else "[✗]"
        print(f"{status} {check:20} {'PASS' if passed else 'FAIL'}")
    
    all_passed = all(results.values())
    
    print("\n" + "="*60)
    
    if all_passed:
        print("✓ ALL CHECKS PASSED - Ready to teleoperate!")
        print("\nTo start teleoperation:")
        print("  ./teleoperate_keyboard.py")
        return 0
    else:
        print("✗ Some checks failed - Fix the issues above")
        print("\nTo install dependencies, run:")
        print("  python3 setup_utils.py --install")
        return 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--install":
        if install_dependencies():
            print("\n[✓] Dependencies installed successfully")
            sys.exit(0)
        else:
            print("\n[✗] Failed to install some dependencies")
            sys.exit(1)
    else:
        sys.exit(main())
